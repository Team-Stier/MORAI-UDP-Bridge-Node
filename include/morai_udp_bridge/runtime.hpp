#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>

#include "morai_udp_bridge/protocol.hpp"

namespace morai_udp_bridge {

using SteadyClock = std::chrono::steady_clock;

struct Datagram {
  std::vector<std::uint8_t> bytes;
  std::string sender;
  ros::Time ros_receipt;
  SteadyClock::time_point steady_receipt;
};

struct ChannelSnapshot {
  std::string name;
  std::string direction;
  std::string topic;
  std::uint16_t port{0};
  int level{diagnostic_msgs::DiagnosticStatus::WARN};
  std::string message{"waiting"};
  std::string error_code{"WAITING"};
  ros::Time state_since;
  ros::Time last_error_time;
  ros::Time receipt_time;
  bool has_logical_publish{false};
  SteadyClock::time_point last_logical_publish;
  double stale_timeout{0.0};
  std::uint64_t received{0};
  std::uint64_t published{0};
  std::uint64_t rejected_source{0};
  std::uint64_t parse_error{0};
  std::uint64_t superseded{0};
  std::uint64_t dropped{0};
  std::uint64_t sent{0};
  std::uint64_t send_error{0};
  std::uint64_t sanitized{0};
  std::uint64_t stale_fallback{0};
  double source_sec{0.0};
  double source_nsec{0.0};
  std::size_t requested_buffer{0};
  std::size_t actual_buffer{0};
  std::vector<SteadyClock::time_point> event_times;
  std::vector<double> lifetimes_ms;
  std::map<std::string, std::string> extra;
};

class DiagnosticsReporter {
 public:
  DiagnosticsReporter(ros::NodeHandle node, const std::string& rich_topic,
                      const std::string& high_hz_topic, double period);
  void addChannel(const std::string& name, const std::string& direction,
                  std::uint16_t port, const std::string& topic,
                  double stale_timeout);
  void setSocketBuffer(const std::string& name, std::size_t requested,
                       std::size_t actual);
  void received(const std::string& name, const Datagram& datagram);
  void rejected(const std::string& name, const Datagram& datagram);
  void parseError(const std::string& name, const std::string& detail,
                  const ros::Time& stamp);
  void valueAnomaly(const std::string& name, const std::string& detail,
                    const ros::Time& stamp);
  void configError(const std::string& name, const std::string& detail);
  void socketError(const std::string& name, const std::string& code,
                   const std::string& detail, const ros::Time& stamp);
  void ignored(const std::string& name, const std::string& code,
               const ros::Time& stamp);
  void superseded(const std::string& name, std::size_t count);
  void dropped(const std::string& name, const std::string& reason,
               const ros::Time& stamp);
  void published(const std::string& name, const ros::Time& stamp,
                 SteadyClock::time_point receipt, double source_sec = 0.0,
                 double source_nsec = 0.0);
  void commandAccepted(bool sanitized, const ros::Time& stamp);
  void sent(const std::string& name, const ros::Time& stamp,
            SteadyClock::time_point started);
  void sendError(const std::string& name, const std::string& detail,
                 const ros::Time& stamp);
  void staleFallback(const std::string& name, double age,
                     const ros::Time& stamp);
  void recovered(const std::string& name, const std::string& code,
                 const ros::Time& stamp);
  void workerException(const std::string& name, const std::string& detail,
                       const ros::Time& stamp);
  void setExtra(const std::string& name, const std::string& key,
                const std::string& value);
  void checkStale();

 private:
  void publishHighHzLocked(ChannelSnapshot& channel, const ros::Time& stamp);
  diagnostic_msgs::DiagnosticStatus statusLocked(const ChannelSnapshot& channel,
                                                  bool rich) const;
  void publishRich(const ros::WallTimerEvent&);
  ChannelSnapshot& channel(const std::string& name);

  mutable std::mutex mutex_;
  std::map<std::string, ChannelSnapshot> channels_;
  ros::Publisher rich_publisher_;
  ros::Publisher high_hz_publisher_;
  ros::WallTimer rich_timer_;
};

enum class SocketAction { kRetryImmediately, kKeep, kReopen };

class SocketRecoveryPolicy {
 public:
  explicit SocketRecoveryPolicy(unsigned int threshold) : threshold_(threshold) {}
  SocketAction classify(int error, unsigned int consecutive_errors) const;

 private:
  unsigned int threshold_;
};

class UdpRxWorkerBase {
 public:
  using BatchHandler = std::function<void(std::vector<Datagram>)>;
  using ResetHandler = std::function<void()>;
  UdpRxWorkerBase(std::string name, std::string allowed_source_ip,
                  unsigned int reopen_threshold, BatchHandler handler,
                  ResetHandler reset, DiagnosticsReporter* reporter);
  virtual ~UdpRxWorkerBase();
  void start();
  void stop();

 protected:
  const std::string& name() const { return name_; }
  virtual void ensureSocket() = 0;
  virtual int receiveDatagram(std::uint8_t* buffer, std::size_t capacity,
                              Datagram* datagram, int* error) = 0;
  virtual void resetSocket() = 0;

 private:
  bool drainBatch(unsigned int* consecutive_errors);
  void resetRuntimeState();
  void run();

  std::string name_;
  std::string allowed_source_ip_;
  SocketRecoveryPolicy recovery_;
  BatchHandler handler_;
  ResetHandler reset_;
  DiagnosticsReporter* reporter_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::vector<Datagram> batch_;
};

class UdpRxWorker final : public UdpRxWorkerBase {
 public:
  UdpRxWorker(std::string name, std::string bind_ip, std::uint16_t port,
              std::string allowed_source_ip, std::size_t receive_buffer,
              unsigned int reopen_threshold, BatchHandler handler,
              ResetHandler reset, DiagnosticsReporter* reporter);
  ~UdpRxWorker();

 private:
  int bindSocket();
  void closeSocket();
  void ensureSocket() override;
  int receiveDatagram(std::uint8_t* buffer, std::size_t capacity,
                      Datagram* datagram, int* error) override;
  void resetSocket() override;

  std::string bind_ip_;
  std::uint16_t port_;
  std::size_t receive_buffer_;
  DiagnosticsReporter* reporter_;
  int socket_{-1};
};

class UdpSender {
 public:
  UdpSender(std::string ip, std::uint16_t port, unsigned int reopen_threshold);
  ~UdpSender();
  bool send(const std::vector<std::uint8_t>& packet, int* error);

 private:
  bool open();
  void close();
  std::string ip_;
  std::uint16_t port_;
  unsigned int threshold_;
  unsigned int consecutive_errors_{0};
  int socket_{-1};
  std::vector<std::uint8_t> address_;
};

class ControlUdpSocket {
 public:
  ControlUdpSocket(std::string destination_ip, std::uint16_t destination_port,
                   std::string bind_ip, std::uint16_t source_port,
                   std::size_t receive_buffer, unsigned int reopen_threshold);
  ~ControlUdpSocket();
  bool send(const std::vector<std::uint8_t>& packet, int* error);
  int receive(std::uint8_t* buffer, std::size_t capacity,
              Datagram* datagram, int* error);
  bool ensureOpen();
  bool reopen();
  std::size_t actualReceiveBuffer() const;

 private:
  bool openLocked();
  void closeLocked();

  std::string bind_ip_;
  std::uint16_t source_port_;
  std::size_t receive_buffer_;
  unsigned int threshold_;
  unsigned int consecutive_errors_{0};
  int socket_{-1};
  std::size_t actual_receive_buffer_{0};
  std::vector<std::uint8_t> address_;
  mutable std::mutex mutex_;
};

class CompetitionRxWorker final : public UdpRxWorkerBase {
 public:
  CompetitionRxWorker(std::string name, std::string allowed_source_ip,
                      unsigned int reopen_threshold, BatchHandler handler,
                      ResetHandler reset, DiagnosticsReporter* reporter,
                      ControlUdpSocket* socket);
  ~CompetitionRxWorker();

 private:
  void ensureSocket() override;
  int receiveDatagram(std::uint8_t* buffer, std::size_t capacity,
                      Datagram* datagram, int* error) override;
  void resetSocket() override;

  ControlUdpSocket* socket_;
};

class LatestCommand {
 public:
  void store(const ControlCommand& command, SteadyClock::time_point receipt);
  bool snapshot(ControlCommand* command, double* age_seconds) const;

 private:
  mutable std::mutex mutex_;
  bool has_command_{false};
  ControlCommand command_;
  SteadyClock::time_point receipt_;
};

class CommandFreshnessGuard {
 public:
  CommandFreshnessGuard(double timeout, double safe_brake)
      : timeout_(timeout), safe_brake_(safe_brake) {}
  ControlCommand select(bool has_command, const ControlCommand& latest,
                        double age, bool* stale) const;

 private:
  double timeout_;
  double safe_brake_;
};

}  // namespace morai_udp_bridge
