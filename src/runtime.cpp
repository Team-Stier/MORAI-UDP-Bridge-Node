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

double ChannelFreshnessMonitor::age(bool has_publish,
                                    SteadyClock::time_point last_publish,
                                    SteadyClock::time_point now) {
  return has_publish ? std::chrono::duration<double>(now - last_publish).count()
                     : std::numeric_limits<double>::infinity();
}

bool ChannelFreshnessMonitor::check(double age_seconds, double timeout) {
  return timeout > 0.0 && age_seconds >= timeout;
}

DiagnosticsReporter::DiagnosticsReporter(ros::NodeHandle node,
                                         const std::string& high_hz_topic) {
  high_hz_publisher_ = node.advertise<diagnostic_msgs::DiagnosticArray>(high_hz_topic, 1, false);
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
  item.last_transition_time = item.state_since;
  channels_[name] = item;
}

ChannelSnapshot& DiagnosticsReporter::channel(const std::string& name) {
  const auto found = channels_.find(name);
  if (found == channels_.end()) throw std::logic_error("unknown diagnostic channel: " + name);
  return found->second;
}

void DiagnosticsReporter::transition(ChannelSnapshot& item, int level,
                                      const std::string& code,
                                      const std::string& message,
                                      const ros::Time& stamp) {
  if (item.level != level || item.error_code != code) {
    item.state_since = stamp;
    item.last_transition_time = stamp;
  }
  item.level = level;
  item.error_code = code;
  item.message = message;
  if (level != diagnostic_msgs::DiagnosticStatus::OK) item.last_error_time = stamp;
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
  transition(item, diagnostic_msgs::DiagnosticStatus::WARN, "SOURCE_REJECTED",
             "source rejected", datagram.ros_receipt);
  publishHighHzLocked(item, datagram.ros_receipt);
}

void DiagnosticsReporter::parseError(const std::string& name,
                                     const std::string& detail,
                                     const ros::Time& stamp, std::size_t count) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  item.parse_error += count;
  transition(item, diagnostic_msgs::DiagnosticStatus::WARN, "STRUCTURAL_ERROR", detail, stamp);
  ROS_WARN_STREAM_THROTTLE(1.0, name << ": STRUCTURAL_ERROR: " << detail);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::valueAnomaly(const std::string& name,
                                       const std::string& detail,
                                       const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  item.anomaly = detail;
  if (detail.empty()) return;
  transition(item, diagnostic_msgs::DiagnosticStatus::WARN, "VALUE_ANOMALY", detail, stamp);
  ROS_WARN_STREAM_THROTTLE(1.0, name << ": VALUE_ANOMALY: " << detail);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::recordChunk(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++channel(name).chunks;
}

void DiagnosticsReporter::configError(const std::string& name,
                                      const std::string& detail) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  item.disabled = true;
  transition(item, diagnostic_msgs::DiagnosticStatus::ERROR, "CONFIG_ERROR", detail, ros::Time::now());
  ROS_ERROR_STREAM(name << ": CONFIG_ERROR: " << detail);
  publishHighHzLocked(item, item.last_error_time);
}

void DiagnosticsReporter::socketError(const std::string& name,
                                      const std::string& code,
                                      const std::string& detail,
                                      const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  transition(item, code == "SOCKET_REOPEN" ? diagnostic_msgs::DiagnosticStatus::ERROR
                                            : diagnostic_msgs::DiagnosticStatus::WARN,
             code, detail, stamp);
  ROS_WARN_STREAM_THROTTLE(1.0, name << ": " << code << ": " << detail);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::ignored(const std::string& name, const std::string& code,
                                  const ros::Time& stamp, std::size_t increment) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  const auto found = item.extra.find("non_gga");
  const std::uint64_t count = found == item.extra.end() ? 0U : std::stoull(found->second);
  item.extra["non_gga"] = text(count + increment);
  transition(item, item.level, code, "normal non-target packet", stamp);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::superseded(const std::string& name, std::size_t count) {
  std::lock_guard<std::mutex> lock(mutex_);
  channel(name).superseded += count;
}

void DiagnosticsReporter::recordSelection(const std::string& name,
                                           std::size_t rejected,
                                           std::size_t non_target) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& item = channel(name);
  item.parse_error += rejected;
  if (name == "gps") {
    auto& count = item.extra["non_gga"];
    count = text((count.empty() ? 0U : std::stoull(count)) + non_target);
  }
}

void DiagnosticsReporter::dropped(const std::string& name,
                                  const std::string& reason,
                                  const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.dropped;
  ++item.dropped_frames;
  if (reason == "JPEG_STRUCTURE") ++item.invalid_frames;
  transition(item, diagnostic_msgs::DiagnosticStatus::WARN, "FRAME_DROPPED", reason, stamp);
  ROS_WARN_STREAM_THROTTLE(1.0, name << ": FRAME_DROPPED: " << reason);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::published(const std::string& name,
                                    const ros::Time& stamp,
                                    SteadyClock::time_point,
                                    double source_sec,
                                    double source_nsec) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.published;
  if (name.compare(0, 7, "camera_") == 0) ++item.completed_frames;
  const bool recovering = item.stale_sensor;
  item.stale_sensor = false;
  item.receipt_time = stamp;
  item.source_sec = source_sec;
  item.source_nsec = source_nsec;
  item.has_logical_publish = true;
  item.last_logical_publish = SteadyClock::now();
  if (recovering) {
    transition(item, diagnostic_msgs::DiagnosticStatus::OK, "OK_RECOVERY", "new sensor sample", stamp);
    publishHighHzLocked(item, stamp);
  }
  transition(item, item.anomaly.empty() ? diagnostic_msgs::DiagnosticStatus::OK
                                       : diagnostic_msgs::DiagnosticStatus::WARN,
             item.anomaly.empty() ? "PUBLISHED" : "VALUE_ANOMALY",
             item.anomaly.empty() ? "published" : item.anomaly, stamp);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::commandAccepted(bool sanitized, const ros::Time& stamp,
                                           SteadyClock::time_point receipt,
                                           const std::string& before,
                                           const std::string& after,
                                           const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  const bool was_sanitized = item.sanitized_command;
  item.sanitized_command = sanitized;
  item.has_command = true;
  item.last_command = receipt;
  item.receipt_time = stamp;
  item.extra["before"] = before;
  item.extra["after"] = after;
  if (sanitized) {
    ++item.sanitized;
    const std::string code = name == "lamp_tx" ? "LAMP_VALUE_ANOMALY" : "SANITIZED_COMMAND";
    transition(item, diagnostic_msgs::DiagnosticStatus::WARN, code,
               "before=" + before + " after=" + after, stamp);
    ROS_WARN_STREAM_THROTTLE(1.0, name << ": " << code << " before=" << before << " after=" << after);
  } else {
    transition(item, diagnostic_msgs::DiagnosticStatus::OK,
               was_sanitized ? "SANITIZED_COMMAND_RECOVERED" : "COMMAND_ACCEPTED",
               "command accepted", stamp);
  }
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::sent(const std::string& name, const ros::Time& stamp,
                               SteadyClock::time_point) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.sent;
  if (item.stale_command) {
    transition(item, diagnostic_msgs::DiagnosticStatus::WARN, "STALE_COMMAND",
               "stale command, safe brake selected", stamp);
  } else if (item.sanitized_command) {
    transition(item, diagnostic_msgs::DiagnosticStatus::WARN,
               name == "lamp_tx" ? "LAMP_VALUE_ANOMALY" : "SANITIZED_COMMAND",
               "before=" + item.extra["before"] + " after=" + item.extra["after"], stamp);
  } else {
    transition(item, diagnostic_msgs::DiagnosticStatus::OK, "SENT", "sent", stamp);
  }
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::sendError(const std::string& name,
                                    const std::string& detail,
                                    const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  ++item.send_error;
  transition(item, diagnostic_msgs::DiagnosticStatus::ERROR,
             name == "lamp_tx" ? "LAMP_TX_SEND_ERROR" : "TX_SEND_ERROR", detail, stamp);
  ROS_WARN_STREAM_THROTTLE(1.0, name << ": send error: " << detail);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::staleFallback(const std::string& name, double age,
                                        const ros::Time& stamp, bool stale) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  const bool changed = item.stale_command != stale;
  item.stale_command = stale;
  if (stale) ++item.stale_fallback;
  if (!changed) return;
  transition(item, stale ? diagnostic_msgs::DiagnosticStatus::WARN : diagnostic_msgs::DiagnosticStatus::OK,
             stale ? "STALE_COMMAND" : "STALE_COMMAND_RECOVERED",
             stale ? "stale command, safe brake selected" : "fresh command restored", stamp);
  ROS_WARN_STREAM(name << ": " << item.error_code << " age=" << age);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::recovered(const std::string& name,
                                    const std::string& code,
                                    const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  if (item.level == diagnostic_msgs::DiagnosticStatus::OK) return;
  transition(item, diagnostic_msgs::DiagnosticStatus::OK, code, "recovered", stamp);
  publishHighHzLocked(item, stamp);
}

void DiagnosticsReporter::workerException(const std::string& name,
                                          const std::string& detail,
                                          const ros::Time& stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelSnapshot& item = channel(name);
  transition(item, diagnostic_msgs::DiagnosticStatus::ERROR, "WORKER_EXCEPTION", detail, stamp);
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
    if (item.direction != "rx" || item.disabled) continue;
    const double age = ChannelFreshnessMonitor::age(item.has_logical_publish, item.last_logical_publish, now);
    if (ChannelFreshnessMonitor::check(age, item.stale_timeout) && !item.stale_sensor) {
      item.stale_sensor = true;
      transition(item, diagnostic_msgs::DiagnosticStatus::WARN, "STALE", "channel stale; no republish", stamp);
      publishHighHzLocked(item, stamp);
    }
  }
}

diagnostic_msgs::DiagnosticStatus DiagnosticsReporter::statusLocked(
    const ChannelSnapshot& item) const {
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
  status.values.push_back(value("last_transition_time", decimal(item.last_transition_time.toSec())));
  const auto now = SteadyClock::now();
  if (item.direction == "rx") {
    status.values.push_back(value("stale_age_ms", decimal(1000.0 * ChannelFreshnessMonitor::age(
        item.has_logical_publish, item.last_logical_publish, now))));
  } else {
    status.values.push_back(value("command_age_ms", decimal(1000.0 * ChannelFreshnessMonitor::age(
        item.has_command, item.last_command, now))));
  }
  if (item.has_logical_publish && item.name != "gps") {
    status.values.push_back(value("last_source_sec", text(item.source_sec)));
    status.values.push_back(value("last_source_nsec", text(item.source_nsec)));
  }
  for (const auto& extra : item.extra) {
    status.values.push_back(value(extra.first, extra.second));
  }

  status.values.push_back(value("state_since", decimal(item.state_since.toSec())));
  status.values.push_back(value("last_error_time", decimal(item.last_error_time.toSec())));
  if (item.name.compare(0, 7, "camera_") == 0) {
    status.values.insert(status.values.end(), {
        value("chunks", text(item.chunks)), value("completed_frames", text(item.completed_frames)),
        value("dropped_frames", text(item.dropped_frames)), value("invalid_frames", text(item.invalid_frames)),
        value("parse_error_count", text(item.parse_error))});
  }
  status.values.insert(status.values.end(), {
      value("received", text(item.received)), value("published", text(item.published)),
      value("rejected_source", text(item.rejected_source)), value("parse_error", text(item.parse_error)),
      value("superseded", text(item.superseded)), value("dropped", text(item.dropped)),
      value("sent", text(item.sent)), value("send_error", text(item.send_error)),
      value("sanitized", text(item.sanitized)), value("stale_fallback", text(item.stale_fallback)),
      value("requested_receive_buffer_bytes", text(item.requested_buffer)),
      value("actual_receive_buffer_bytes", text(item.actual_buffer))});
  return status;
}

void DiagnosticsReporter::publishHighHzLocked(ChannelSnapshot& item,
                                              const ros::Time& stamp) {
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(statusLocked(item));
  high_hz_publisher_.publish(array);
}

SocketAction SocketRecoveryPolicy::classify(int error,
                                            unsigned int consecutive_errors) const {
  if (error == EINTR) return SocketAction::kRetryImmediately;
  if (fatalSocketError(error) || consecutive_errors >= threshold_) return SocketAction::kReopen;
  return SocketAction::kKeep;
}

SocketAction TxSocketRecoveryPolicy::classify(int error,
                                              unsigned int consecutive_errors) const {
  return fatalSocketError(error) || consecutive_errors >= threshold_
             ? SocketAction::kReopen : SocketAction::kKeep;
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
  ROS_INFO("%s receive buffer: requested=%zu actual=%d", name_.c_str(), receive_buffer_, actual);
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
  consecutive_errors_ = 0;
  reset_();
}

void UdpRxWorker::start() { thread_ = std::thread(&UdpRxWorker::run, this); }

void UdpRxWorker::stop() {
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
}

bool UdpRxWorker::drainBatch(std::array<std::uint8_t, 65535U>& buffer) {
  while (!stop_.load()) {
    sockaddr_in sender{};
    socklen_t sender_size = sizeof(sender);
    const ssize_t count = recvfrom(socket_, buffer.data(), buffer.size(), 0,
                                   reinterpret_cast<sockaddr*>(&sender), &sender_size);
    if (count >= 0) {
      consecutive_errors_ = 0;
      Datagram datagram;
      datagram.ros_receipt = ros::Time::now();
      datagram.steady_receipt = SteadyClock::now();
      datagram.bytes.assign(buffer.begin(), buffer.begin() + count);
      datagram.sender = senderIp(sender);
      reporter_->received(name_, datagram);
      if (UdpSourceFilter::accept(datagram.sender, allowed_source_ip_)) {
        batch_.push_back(std::move(datagram));
        if (name_.compare(0, 7, "camera_") == 0 && batch_.size() >= 16U) return true;
      } else {
        reporter_->rejected(name_, datagram);
      }
      continue;
    }
    const int error = errno;
    if (error == EAGAIN || error == EWOULDBLOCK) return true;
    if (error != EINTR) ++consecutive_errors_;
    const SocketAction action = recovery_.classify(error, consecutive_errors_);
    if (action == SocketAction::kRetryImmediately) continue;
    reporter_->socketError(name_, action == SocketAction::kReopen ? "SOCKET_REOPEN" : "RECV_ERROR",
                           std::string("recvfrom: ") + std::strerror(error), ros::Time::now());
    if (action == SocketAction::kReopen) {
      resetRuntimeState();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }
  return false;
}

void UdpRxWorker::run() {
  std::array<std::uint8_t, 65535U> buffer{};
  std::chrono::milliseconds backoff(100);
  while (!stop_.load() && ros::ok()) {
    try {
      if (socket_ < 0) {
        socket_ = bindSocket();
        consecutive_errors_ = 0U;
        backoff = std::chrono::milliseconds(100);
      }
      const bool drained = drainBatch(buffer);
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
      ROS_ERROR_THROTTLE(1.0, "%s worker exception: %s", name_.c_str(), error.what());
      resetRuntimeState();
      std::this_thread::sleep_for(backoff);
      backoff = std::min(backoff * 2, std::chrono::milliseconds(2000));
    }
  }
  closeSocket();
}

UdpSender::UdpSender(std::string ip, std::uint16_t port)
    : ip_(std::move(ip)), port_(port) {
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (inet_pton(AF_INET, ip_.c_str(), &address.sin_addr) != 1)
    throw std::invalid_argument("invalid destination IP: " + ip_);
  address_.resize(sizeof(address));
  std::memcpy(address_.data(), &address, sizeof(address));
  open();
}

UdpSender::~UdpSender() { close(); }

bool UdpSender::open() {
  close();
  socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  return socket_ >= 0;
}

bool UdpSender::reopen() {
  consecutive_errors_ = 0;
  return open();
}

void UdpSender::close() {
  if (socket_ >= 0) ::close(socket_);
  socket_ = -1;
}

bool UdpSender::send(const std::vector<std::uint8_t>& packet, int* error) {
  if (socket_ < 0) {
    *error = EBADF;
    ++consecutive_errors_;
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
