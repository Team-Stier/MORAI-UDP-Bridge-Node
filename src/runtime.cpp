#include "morai_udp_bridge/runtime.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <diagnostic_msgs/KeyValue.h>

namespace morai_udp_bridge {
namespace {

diagnostic_msgs::KeyValue value(const std::string& key, const std::string& text) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = text;
  return item;
}

template <typename T>
std::string text(T input) {
  std::ostringstream output;
  output << input;
  return output.str();
}

std::string decimal(double input) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(3) << input;
  return output.str();
}

std::string senderIp(const sockaddr_in& sender) {
  std::array<char, INET_ADDRSTRLEN> buffer{};
  return inet_ntop(AF_INET, &sender.sin_addr, buffer.data(), buffer.size()) ? buffer.data() : "";
}

bool fatalSocketError(int error) {
  return error == EBADF || error == ENOTSOCK || error == EINVAL;
}

}  // namespace

DiagnosticsReporter::DiagnosticsReporter(ros::NodeHandle node,
                                         const std::string& rich_topic,
                                         const std::string& high_hz_topic,
                                         double period) {
  rich_publisher_ = node.advertise<diagnostic_msgs::DiagnosticArray>(rich_topic, 1, false);
  high_hz_publisher_ = node.advertise<diagnostic_msgs::DiagnosticArray>(high_hz_topic, 1, false);
  rich_timer_ = node.createWallTimer(ros::WallDuration(period),
                                     &DiagnosticsReporter::publishRich, this);
}

void DiagnosticsReporter::addChannel(const std::string& name,
                                     const std::string& direction,
                                     std::uint16_t port,
                                     const std::string& topic,
                                     double stale_timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot item;
  item.name = name;
  item.direction = direction;
  item.port = port;
  item.topic = topic;
  item.stale_timeout = stale_timeout;
  item.state_since = ros::Time::now();
  channels_[name] = item;
}

ChannelSnapshot& DiagnosticsReporter::channel(const std::string& name) {
  const auto found = channels_.find(name);
  if (found == channels_.end()) throw std::logic_error("unknown diagnostic channel: " + name);
  return found->second;
}

void DiagnosticsReporter::setSocketBuffer(const std::string& name,
                                          std::size_t requested,
                                          std::size_t actual) {
  std::lock_guard<std::mutex> lock(mutex_);
  channel(name).requested_buffer = requested;
  channel(name).actual_buffer = actual;
}

void DiagnosticsReporter::received(const std::string& name, const Datagram& datagram) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.received;
  item.receipt_time = datagram.ros_receipt;
  item.extra["last_sender"] = datagram.sender;
}

void DiagnosticsReporter::rejected(const std::string& name, const Datagram& datagram) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.rejected_source;
  item.level = diagnostic_msgs::DiagnosticStatus::WARN;
  item.message = "source rejected";
  item.error_code = "SOURCE_REJECTED";
  item.last_error_time = datagram.ros_receipt;
  publishHighHzLocked(item, datagram.ros_receipt);
}

void DiagnosticsReporter::parseError(const std::string& name,
                                     const std::string& detail,
                                     const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.parse_error;
  item.level = diagnostic_msgs::DiagnosticStatus::WARN;
  item.message = detail;
  item.error_code = "STRUCTURAL_ERROR";
  item.last_error_time = stamp;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::valueAnomaly(const std::string& name,
                                       const std::string& detail,
                                       const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  item.level = diagnostic_msgs::DiagnosticStatus::WARN;
  item.message = detail;
  item.error_code = "VALUE_ANOMALY";
  item.last_error_time = stamp;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::configError(const std::string& name,
                                      const std::string& detail) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  item.level = diagnostic_msgs::DiagnosticStatus::ERROR;
  item.message = detail;
  item.error_code = "CONFIG_ERROR";
  item.last_error_time = ros::Time::now();
  publishHighHzLocked(item, item.last_error_time);
}

void DiagnosticsReporter::socketError(const std::string& name,
                                      const std::string& code,
                                      const std::string& detail,
                                      const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  item.level = code == "SOCKET_REOPEN" ? diagnostic_msgs::DiagnosticStatus::ERROR
                                        : diagnostic_msgs::DiagnosticStatus::WARN;
  item.message = detail;
  item.error_code = code;
  item.last_error_time = stamp;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::ignored(const std::string& name, const std::string& code,
                                  const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  const auto found = item.extra.find("non_gga");
  const std::uint64_t count = found == item.extra.end() ? 0U : std::stoull(found->second);
  item.extra["non_gga"] = text(count + 1U);
  item.error_code = code;
  item.message = "normal non-target packet";
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::superseded(const std::string& name, std::size_t count) {
  std::lock_guard<std::mutex> lock(mutex_);
  channel(name).superseded += count;
}

void DiagnosticsReporter::dropped(const std::string& name,
                                  const std::string& reason,
                                  const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.dropped;
  item.level = diagnostic_msgs::DiagnosticStatus::WARN;
  item.message = reason;
  item.error_code = "FRAME_DROPPED";
  item.last_error_time = stamp;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::published(const std::string& name,
                                    const ros::Time& stamp,
                                    SteadyClock::time_point receipt,
                                    double source_sec,
                                    double source_nsec) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.published;
  item.has_logical_publish = true;
  item.last_logical_publish = SteadyClock::now();
  item.event_times.push_back(item.last_logical_publish);
  item.lifetimes_ms.push_back(
      std::chrono::duration<double, std::milli>(SteadyClock::now() - receipt).count());
  if (item.event_times.size() > 512U) item.event_times.erase(item.event_times.begin());
  if (item.lifetimes_ms.size() > 512U) item.lifetimes_ms.erase(item.lifetimes_ms.begin());
  item.level = diagnostic_msgs::DiagnosticStatus::OK;
  item.message = "published";
  item.error_code = "PUBLISHED";
  item.receipt_time = stamp;
  item.source_sec = source_sec;
  item.source_nsec = source_nsec;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::commandAccepted(bool sanitized, const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel("control_tx");
  if (sanitized) {
    ++item.sanitized;
    item.level = diagnostic_msgs::DiagnosticStatus::WARN;
    item.message = "command sanitized";
    item.error_code = "SANITIZED_COMMAND";
    item.last_error_time = stamp;
  } else {
    item.level = diagnostic_msgs::DiagnosticStatus::OK;
    item.message = "command accepted";
    item.error_code = "COMMAND_ACCEPTED";
  }
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::sent(const std::string& name, const ros::Time& stamp,
                               SteadyClock::time_point started) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.sent;
  item.event_times.push_back(SteadyClock::now());
  item.lifetimes_ms.push_back(
      std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count());
  item.level = diagnostic_msgs::DiagnosticStatus::OK;
  item.message = "sent";
  item.error_code = "SENT";
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::sendError(const std::string& name,
                                    const std::string& detail,
                                    const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.send_error;
  item.level = diagnostic_msgs::DiagnosticStatus::ERROR;
  item.message = detail;
  item.error_code = "TX_SEND_ERROR";
  item.last_error_time = stamp;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::staleFallback(const std::string& name, double age,
                                        const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.stale_fallback;
  item.level = diagnostic_msgs::DiagnosticStatus::WARN;
  item.message = "stale command, safe brake selected";
  item.error_code = "STALE_COMMAND";
  item.extra["command_age_ms"] = decimal(age * 1000.0);
  item.last_error_time = stamp;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::recovered(const std::string& name,
                                    const std::string& code,
                                    const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  if (item.level == diagnostic_msgs::DiagnosticStatus::OK) return;
  item.level = diagnostic_msgs::DiagnosticStatus::OK;
  item.message = "recovered";
  item.error_code = code;
  item.state_since = stamp;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::workerException(const std::string& name,
                                          const std::string& detail,
                                          const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  item.level = diagnostic_msgs::DiagnosticStatus::ERROR;
  item.message = detail;
  item.error_code = "WORKER_EXCEPTION";
  item.last_error_time = stamp;
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::setExtra(const std::string& name,
                                   const std::string& key,
                                   const std::string& input) {
  std::lock_guard<std::mutex> lock(mutex_);
  channel(name).extra[key] = input;
}

void DiagnosticsReporter::checkStale() {
  const auto now = SteadyClock::now();
  const ros::Time stamp = ros::Time::now();
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& entry : channels_) {
    ChannelSnapshot& item = entry.second;
    if (item.direction != "rx" || item.stale_timeout <= 0.0) continue;
    const double age = item.has_logical_publish
                           ? std::chrono::duration<double>(now - item.last_logical_publish).count()
                           : std::numeric_limits<double>::infinity();
    if (age > item.stale_timeout && item.error_code != "STALE") {
      item.level = diagnostic_msgs::DiagnosticStatus::WARN;
      item.message = "channel stale; no republish";
      item.error_code = "STALE";
      item.last_error_time = stamp;
      item.extra["stale_age_ms"] = std::isfinite(age) ? decimal(age * 1000.0) : "inf";
      publishHighHzLocked(item, stamp);
    }
  }
}

diagnostic_msgs::DiagnosticStatus DiagnosticsReporter::statusLocked(
    const ChannelSnapshot& item, bool rich) const {
  diagnostic_msgs::DiagnosticStatus status;
  status.name = "morai_udp_bridge/" + item.name;
  status.hardware_id = "udp:" + text(item.port);
  status.level = item.level;
  status.message = item.message;
  status.values.push_back(value("direction", item.direction));
  status.values.push_back(value("channel", item.name));
  status.values.push_back(value("port", text(item.port)));
  status.values.push_back(value("topic", item.topic));
  status.values.push_back(value("error_code", item.error_code));
  status.values.push_back(value("receipt_time", decimal(item.receipt_time.toSec())));
  if (item.source_sec != 0.0 || item.source_nsec != 0.0) {
    status.values.push_back(value("last_source_sec", text(item.source_sec)));
    status.values.push_back(value("last_source_nsec", text(item.source_nsec)));
  }
  for (const auto& extra : item.extra) status.values.push_back(value(extra.first, extra.second));
  if (!rich) return status;

  const auto now = SteadyClock::now();
  std::size_t recent = 0U;
  SteadyClock::time_point first;
  for (const auto& event : item.event_times) {
    if (std::chrono::duration<double>(now - event).count() <= 5.0) {
      if (recent == 0U) first = event;
      ++recent;
    }
  }
  double hz = 0.0;
  if (recent > 1U) hz = (recent - 1U) / std::chrono::duration<double>(item.event_times.back() - first).count();
  double p95 = 0.0;
  if (!item.lifetimes_ms.empty()) {
    std::vector<double> sorted = item.lifetimes_ms;
    std::sort(sorted.begin(), sorted.end());
    p95 = sorted[static_cast<std::size_t>(std::ceil(sorted.size() * 0.95)) - 1U];
  }
  status.values.insert(status.values.end(), {
      value("received", text(item.received)), value("published", text(item.published)),
      value("rejected_source", text(item.rejected_source)), value("parse_error", text(item.parse_error)),
      value("superseded", text(item.superseded)), value("dropped", text(item.dropped)),
      value("sent", text(item.sent)), value("send_error", text(item.send_error)),
      value("sanitized", text(item.sanitized)), value("stale_fallback", text(item.stale_fallback)),
      value("hz", decimal(hz)), value("lifetime_p95_ms", decimal(p95)),
      value("requested_receive_buffer_bytes", text(item.requested_buffer)),
      value("actual_receive_buffer_bytes", text(item.actual_buffer))});
  return status;
}

void DiagnosticsReporter::publishHighHzLocked(ChannelSnapshot& item,
                                              const ros::Time& stamp) {
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(statusLocked(item, false));
  high_hz_publisher_.publish(array);
}

void DiagnosticsReporter::publishRich(const ros::WallTimerEvent&) {
  checkStale();
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = ros::Time::now();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& entry : channels_) array.status.push_back(statusLocked(entry.second, true));
  rich_publisher_.publish(array);
}

SocketAction SocketRecoveryPolicy::classify(int error,
                                            unsigned int consecutive_errors) const {
  if (error == EINTR) return SocketAction::kRetryImmediately;
  if (fatalSocketError(error) || consecutive_errors >= threshold_) return SocketAction::kReopen;
  return SocketAction::kKeep;
}

UdpRxWorker::UdpRxWorker(std::string name, std::string bind_ip,
                         std::uint16_t port, std::string allowed_source_ip,
                         std::size_t receive_buffer,
                         unsigned int reopen_threshold, BatchHandler handler,
                         ResetHandler reset, DiagnosticsReporter* reporter)
    : name_(std::move(name)), bind_ip_(std::move(bind_ip)), port_(port),
      allowed_source_ip_(std::move(allowed_source_ip)), receive_buffer_(receive_buffer),
      recovery_(reopen_threshold), handler_(std::move(handler)), reset_(std::move(reset)),
      reporter_(reporter) {
  if (!handler_ || !reset_ || !reporter_) throw std::invalid_argument("invalid RX worker dependency");
}

UdpRxWorker::~UdpRxWorker() { stop(); }

int UdpRxWorker::bindSocket() {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    const int error = errno;
    ::close(fd);
    throw std::runtime_error(std::string("fcntl: ") + std::strerror(error));
  }
  const int requested = static_cast<int>(receive_buffer_);
  if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &requested, sizeof(requested)) != 0) {
    const int error = errno;
    ::close(fd);
    throw std::runtime_error(std::string("SO_RCVBUF: ") + std::strerror(error));
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (bind_ip_.empty() || bind_ip_ == "0.0.0.0") address.sin_addr.s_addr = INADDR_ANY;
  else if (inet_pton(AF_INET, bind_ip_.c_str(), &address.sin_addr) != 1) {
    ::close(fd);
    throw std::runtime_error("invalid bind_ip: " + bind_ip_);
  }
  if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    const int error = errno;
    ::close(fd);
    throw std::runtime_error(std::string("bind: ") + std::strerror(error));
  }
  int actual = 0;
  socklen_t length = sizeof(actual);
  getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &actual, &length);
#ifdef __linux__
  actual /= 2;
#endif
  reporter_->setSocketBuffer(name_, receive_buffer_, static_cast<std::size_t>(std::max(actual, 0)));
  return fd;
}

void UdpRxWorker::closeSocket() {
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

void UdpRxWorker::resetRuntimeState() {
  closeSocket();
  batch_.clear();
  reset_();
}

void UdpRxWorker::start() { thread_ = std::thread(&UdpRxWorker::run, this); }

void UdpRxWorker::stop() {
  stop_.store(true);
  closeSocket();
  if (thread_.joinable()) thread_.join();
}

void UdpRxWorker::run() {
  std::array<std::uint8_t, 65535U> buffer{};
  unsigned int consecutive_errors = 0U;
  std::chrono::milliseconds backoff(100);
  while (!stop_.load() && ros::ok()) {
    try {
      if (socket_ < 0) {
        socket_ = bindSocket();
        consecutive_errors = 0U;
        backoff = std::chrono::milliseconds(100);
      }
      bool drained = false;
      while (!stop_.load()) {
        sockaddr_in sender{};
        socklen_t sender_size = sizeof(sender);
        const ssize_t count = recvfrom(socket_, buffer.data(), buffer.size(), 0,
                                       reinterpret_cast<sockaddr*>(&sender), &sender_size);
        if (count >= 0) {
          consecutive_errors = 0U;
          Datagram datagram;
          datagram.bytes.assign(buffer.begin(), buffer.begin() + count);
          datagram.sender = senderIp(sender);
          datagram.ros_receipt = ros::Time::now();
          datagram.steady_receipt = SteadyClock::now();
          reporter_->received(name_, datagram);
          if (!allowed_source_ip_.empty() && datagram.sender != allowed_source_ip_) {
            reporter_->rejected(name_, datagram);
          } else {
            batch_.push_back(std::move(datagram));
          }
          continue;
        }
        const int error = errno;
        if (error == EAGAIN || error == EWOULDBLOCK) {
          drained = true;
          break;
        }
        ++consecutive_errors;
        const SocketAction action = recovery_.classify(error, consecutive_errors);
        if (action == SocketAction::kRetryImmediately) continue;
        reporter_->socketError(name_, action == SocketAction::kReopen ? "SOCKET_REOPEN" : "RECV_ERROR",
                               std::string("recvfrom: ") + std::strerror(error), ros::Time::now());
        if (action == SocketAction::kReopen) resetRuntimeState();
        break;
      }
      if (drained && !batch_.empty()) {
        std::vector<Datagram> batch;
        batch.swap(batch_);
        handler_(std::move(batch));
      } else if (batch_.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (const std::bad_alloc&) {
      throw;
    } catch (const std::exception& error) {
      reporter_->workerException(name_, error.what(), ros::Time::now());
      ROS_ERROR("%s worker exception: %s", name_.c_str(), error.what());
      resetRuntimeState();
      std::this_thread::sleep_for(backoff);
      backoff = std::min(backoff * 2, std::chrono::milliseconds(2000));
    }
  }
  closeSocket();
}

UdpSender::UdpSender(std::string ip, std::uint16_t port,
                     unsigned int reopen_threshold)
    : ip_(std::move(ip)), port_(port), threshold_(reopen_threshold) {
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (inet_pton(AF_INET, ip_.c_str(), &address.sin_addr) != 1)
    throw std::invalid_argument("invalid destination IP: " + ip_);
  address_.resize(sizeof(address));
  std::memcpy(address_.data(), &address, sizeof(address));
  if (!open()) throw std::runtime_error("cannot create UDP sender");
}

UdpSender::~UdpSender() { close(); }

bool UdpSender::open() {
  close();
  socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  return socket_ >= 0;
}

void UdpSender::close() {
  if (socket_ >= 0) ::close(socket_);
  socket_ = -1;
}

bool UdpSender::send(const std::vector<std::uint8_t>& packet, int* error) {
  if (socket_ < 0 && !open()) {
    *error = errno;
    return false;
  }
  const ssize_t sent = sendto(socket_, packet.data(), packet.size(), 0,
                              reinterpret_cast<const sockaddr*>(address_.data()),
                              static_cast<socklen_t>(address_.size()));
  if (sent == static_cast<ssize_t>(packet.size())) {
    consecutive_errors_ = 0U;
    return true;
  }
  *error = sent < 0 ? errno : EMSGSIZE;
  ++consecutive_errors_;
  if (fatalSocketError(*error) || consecutive_errors_ >= threshold_) open();
  return false;
}

void LatestCommand::store(const ControlCommand& command,
                          SteadyClock::time_point receipt) {
  std::lock_guard<std::mutex> lock(mutex_);
  command_ = command;
  receipt_ = receipt;
  has_command_ = true;
}

bool LatestCommand::snapshot(ControlCommand* command, double* age_seconds) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_command_) {
    *age_seconds = std::numeric_limits<double>::infinity();
    return false;
  }
  *command = command_;
  *age_seconds = std::chrono::duration<double>(SteadyClock::now() - receipt_).count();
  return true;
}

ControlCommand CommandFreshnessGuard::select(bool has_command,
                                             const ControlCommand& latest,
                                             double age, bool* stale) const {
  *stale = !has_command || age >= timeout_;
  if (!*stale) return latest;
  ControlCommand safe;
  safe.brake = safe_brake_;
  return safe;
}

}  // namespace morai_udp_bridge
