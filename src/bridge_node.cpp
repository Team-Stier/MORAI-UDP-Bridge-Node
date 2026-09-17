#include <cmath>
#include <arpa/inet.h>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

#include <interfaces/CompetitionVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/Lamps.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "morai_udp_bridge/protocol.hpp"
#include "morai_udp_bridge/runtime.hpp"

namespace morai_udp_bridge {
namespace {

template <typename T>
T requiredParam(const ros::NodeHandle& node, const std::string& name) {
  T value;
  if (!node.getParam(name, value)) throw std::runtime_error("missing parameter: " + name);
  return value;
}

template <typename T>
T param(const ros::NodeHandle& node, const std::string& name, const T& fallback) {
  T value = fallback;
  node.param(name, value, fallback);
  return value;
}

class ChannelConfigValidator {
 public:
  static void validatePort(int port) {
    if (port <= 0 || port > 65535) throw std::runtime_error("port is outside 1..65535");
  }
  static void validatePositive(double value, const std::string& name) {
    if (!std::isfinite(value) || value <= 0.0) throw std::runtime_error(name + " must be positive");
  }
  static void validateTopic(const std::string& topic) {
    std::string error;
    if (topic.empty() || !ros::names::validate(topic, error))
      throw std::runtime_error("invalid topic: " + topic + " " + error);
  }
  static void validateFrame(const std::string& frame) {
    if (frame.empty()) throw std::runtime_error("frame_id is empty");
  }
  static void validateIp(const std::string& ip, bool allow_empty = false) {
    in_addr address{};
    if (allow_empty && ip.empty()) return;
    if (inet_pton(AF_INET, ip.c_str(), &address) != 1)
      throw std::runtime_error("invalid IPv4 address: " + ip);
  }
  static void validateRx(const ros::NodeHandle& config, int port,
                         const std::string& topic, double stale_timeout) {
    validatePort(port);
    validateTopic(topic);
    validatePositive(stale_timeout, "stale_timeout");
    validatePositive(param(config, "receive_buffer_bytes", 4194304), "receive_buffer_bytes");
    validatePositive(param(config, "recv_error_reopen_threshold", 3), "recv_error_reopen_threshold");
    validateIp(param(config, "bind_ip", std::string("0.0.0.0")), true);
    validateIp(param(config, "allowed_source_ip", std::string()), true);
  }
  static void validateTx(const ros::NodeHandle& config, const std::string& prefix) {
    validatePort(requiredParam<int>(config, prefix + "/destination_port"));
    validateTopic(requiredParam<std::string>(config, prefix + "/command_topic"));
    validateIp(requiredParam<std::string>(config, prefix + "/destination_ip"));
    validatePositive(requiredParam<int>(config, prefix + "/send_error_reopen_threshold"), "send_error_reopen_threshold");
    if (prefix == "control") {
      validatePositive(requiredParam<double>(config, "control/send_rate_hz"), "send_rate_hz");
      validatePositive(requiredParam<double>(config, "control/command_timeout_sec"), "command_timeout_sec");
      const double brake = requiredParam<double>(config, "control/safe_brake_command");
      if (!std::isfinite(brake) || brake < 0.0 || brake > 1.0)
        throw std::runtime_error("safe_brake_command is outside 0..1");
    }
  }
};

struct CameraParser {
  static CameraPacket decodeStructure(const Datagram& packet, PacketLayout layout) {
    return decodeCameraPacket(packet.bytes.data(), packet.bytes.size(), layout);
  }
};
struct GpsParser {
  static GgaFix decodeStructure(const Datagram& packet) {
    return decodeGgaDatagram(packet.bytes.data(), packet.bytes.size());
  }
};
struct ImuParser {
  static ImuSample decodeStructure(const Datagram& packet, PacketLayout layout) {
    return decodeImuPacket(packet.bytes.data(), packet.bytes.size(), layout);
  }
};
struct StatusParser {
  static VehicleStatusSample decodeStructure(const Datagram& packet) {
    return decodeVehicleStatusPacket(packet.bytes.data(), packet.bytes.size());
  }
};

class RxValueInspector {
 public:
  static std::string camera(const CameraPacket& packet) {
    if (packet.has_timestamp && packet.nsec >= 1000000000U) return "camera nsec is outside range";
    return {};
  }
  static std::string gps(const GgaFix& fix) {
    if (fix.coordinate_anomaly) return "GGA coordinate is non-finite or outside range";
    if (fix.quality < 0 || fix.satellites < 0) return "GGA quality or satellites is negative";
    if ((fix.has_latitude && !std::isfinite(fix.latitude)) ||
        (fix.has_longitude && !std::isfinite(fix.longitude))) return "GGA coordinate is non-finite";
    if (fix.has_hdop && (!std::isfinite(fix.hdop) || fix.hdop < 0)) return "GGA HDOP is suspicious";
    if ((fix.has_altitude_msl && !std::isfinite(fix.altitude_msl)) ||
        (fix.has_geoid_separation && !std::isfinite(fix.geoid_separation))) return "GGA altitude is non-finite";
    return {};
  }
  static std::string imu(const ImuSample& sample) {
    const double values[] = {sample.orientation_w, sample.orientation_x, sample.orientation_y,
                             sample.orientation_z, sample.angular_velocity_x,
                             sample.angular_velocity_y, sample.angular_velocity_z,
                             sample.linear_acceleration_x, sample.linear_acceleration_y,
                             sample.linear_acceleration_z};
    for (double value : values) if (!std::isfinite(value)) return "IMU contains non-finite value";
    const double norm = std::sqrt(sample.orientation_w * sample.orientation_w +
                                  sample.orientation_x * sample.orientation_x +
                                  sample.orientation_y * sample.orientation_y +
                                  sample.orientation_z * sample.orientation_z);
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.1) return "IMU quaternion norm is suspicious";
    if (sample.has_timestamp && sample.nsec >= 1000000000U) return "IMU nsec is outside range";
    return {};
  }
  static std::string status(const VehicleStatusSample& sample) {
    if (!std::isfinite(sample.source_nsec) || sample.source_nsec < 0.0F ||
        sample.source_nsec >= 1000000000.0F)
      return "status nsec is outside range";
    if (sample.control_mode < 1U || sample.control_mode > 2U) return "status control_mode is unknown";
    if (sample.gear > 5U) return "status gear is unknown";
    if (sample.map_data_id < 0 || sample.map_data_id > 19999) return "status map_data_id is outside range";
    const float values[] = {sample.signed_velocity_mps, sample.accel, sample.brake,
      sample.size_x_m, sample.size_y_m, sample.size_z_m, sample.overhang_m,
      sample.wheelbase_m, sample.rear_overhang_m, sample.roll_rad, sample.pitch_rad,
      sample.yaw_rad, sample.velocity_x_mps, sample.angular_velocity_x_radps,
      sample.angular_velocity_y_radps, sample.angular_velocity_z_radps, sample.steering_angle_rad};
    for (float value : values) if (!std::isfinite(value)) return "status contains non-finite value";
    if (sample.accel < 0 || sample.accel > 1 || sample.brake < 0 || sample.brake > 1)
      return "status pedal is outside range";
    return {};
  }
};

class CameraBridge {
 public:
  CameraBridge(ros::NodeHandle node, std::string name, std::string topic,
               std::string info_topic, std::string frame_id, int width, int height,
               double horizontal_fov_deg, PacketLayout layout,
               DiagnosticsReporter* reporter)
      : name_(std::move(name)), frame_id_(std::move(frame_id)), width_(width), height_(height),
        horizontal_fov_deg_(horizontal_fov_deg), layout_(layout), assembler_(256U, std::chrono::milliseconds(200)),
        reporter_(reporter) {
    ChannelConfigValidator::validatePositive(width_, "camera width");
    ChannelConfigValidator::validatePositive(height_, "camera height");
    ChannelConfigValidator::validatePositive(horizontal_fov_deg_, "camera FOV");
    if (horizontal_fov_deg_ >= 180.0) throw std::runtime_error("camera FOV must be below 180 degrees");
    if (frame_id_.empty()) throw std::runtime_error("camera frame_id is empty");
    image_publisher_ = node.advertise<sensor_msgs::CompressedImage>(topic, 1, false);
    info_publisher_ = node.advertise<sensor_msgs::CameraInfo>(info_topic, 1, false);
  }

  void handle(std::vector<Datagram> batch) {
    for (const Datagram& datagram : batch) {
      try {
        CameraPacket packet = CameraParser::decodeStructure(datagram, layout_);
        reporter_->recordChunk(name_);
        const std::string anomaly = RxValueInspector::camera(packet);
        reporter_->valueAnomaly(name_, anomaly, datagram.ros_receipt);
        std::string drop_reason;
        CameraFrame frame;
        const bool complete = assembler_.add(std::move(packet), datagram.steady_receipt,
                                             &frame, &drop_reason, datagram.ros_receipt.toNSec());
        if (!drop_reason.empty()) reporter_->dropped(name_, drop_reason, datagram.ros_receipt);
        if (complete) {
          ros::Time first_receipt;
          first_receipt.fromNSec(frame.first_ros_receipt_ns);
          publish(translateToRosMessage(frame, first_receipt));
          reporter_->published(name_, first_receipt, frame.first_receipt,
                               frame.source_sec, frame.source_nsec);
        }
      } catch (const ProtocolError& error) {
        reporter_->parseError(name_, error.what(), datagram.ros_receipt);
      }
    }
  }

  void reset() { assembler_.reset(); }

 private:
  std::pair<sensor_msgs::CompressedImage, sensor_msgs::CameraInfo> translateToRosMessage(
      const CameraFrame& frame, const ros::Time& receipt) const {
    sensor_msgs::CompressedImage image;
    image.header.stamp = receipt;
    image.header.frame_id = frame_id_;
    image.format = "jpeg";
    image.data = frame.jpeg;
    sensor_msgs::CameraInfo info;
    info.header = image.header;
    info.width = static_cast<std::uint32_t>(width_);
    info.height = static_cast<std::uint32_t>(height_);
    info.distortion_model = "plumb_bob";
    info.D.assign(5U, 0.0);
    const double fov = horizontal_fov_deg_ * 3.14159265358979323846 / 180.0;
    const double focal = width_ / (2.0 * std::tan(fov / 2.0));
    info.K = {{focal, 0.0, width_ / 2.0, 0.0, focal, height_ / 2.0, 0.0, 0.0, 1.0}};
    info.R = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
    info.P = {{focal, 0.0, width_ / 2.0, 0.0, 0.0, focal, height_ / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    return {image, info};
  }

  void publish(const std::pair<sensor_msgs::CompressedImage, sensor_msgs::CameraInfo>& messages) {
    image_publisher_.publish(messages.first);
    info_publisher_.publish(messages.second);
  }

  std::string name_;
  std::string frame_id_;
  int width_;
  int height_;
  double horizontal_fov_deg_;
  PacketLayout layout_;
  CameraFrameAssembler assembler_;
  DiagnosticsReporter* reporter_;
  ros::Publisher image_publisher_;
  ros::Publisher info_publisher_;
};

class GpsBridge {
 public:
  GpsBridge(ros::NodeHandle node, std::string topic, std::string frame_id,
            DiagnosticsReporter* reporter)
      : frame_id_(std::move(frame_id)), reporter_(reporter) {
    publisher_ = node.advertise<sensor_msgs::NavSatFix>(topic, 1, false);
  }

  void handle(std::vector<Datagram> batch) {
    std::size_t rejected = 0U;
    std::size_t non_target = 0U;
    std::size_t selected = 0;
    GgaFix fix;
    const bool found = LatestDatagramSelector::selectNewestDecodable(batch, GpsParser::decodeStructure,
          &fix, &selected, &rejected, &non_target);
    reporter_->recordSelection("gps", rejected, non_target);
    if (found && rejected > 0U)
      reporter_->parseError("gps", "batch rejected=" + std::to_string(rejected), batch.back().ros_receipt, 0U);
    if (found) {
        const Datagram& datagram = batch[selected];
        reporter_->superseded("gps", selected);
        const std::string anomaly = RxValueInspector::gps(fix);
        reporter_->valueAnomaly("gps", anomaly, datagram.ros_receipt);
        publisher_.publish(translateToRosMessage(fix, datagram.ros_receipt));
        reporter_->setExtra("gps", "gga_utc", fix.utc);
        reporter_->setExtra("gps", "quality", std::to_string(fix.quality));
        reporter_->setExtra("gps", "satellites", std::to_string(fix.satellites));
        reporter_->setExtra("gps", "hdop", fix.has_hdop ? std::to_string(fix.hdop) : "nan");
        reporter_->published("gps", datagram.ros_receipt, datagram.steady_receipt);
        return;
    }
    const ros::Time stamp = batch.back().ros_receipt;
    if (non_target == batch.size()) reporter_->ignored("gps", "NON_GGA", stamp, 0U);
    else reporter_->parseError("gps", "no structurally decodable GGA; rejected=" + std::to_string(rejected), stamp, 0U);
  }

  void reset() {}

 private:
  sensor_msgs::NavSatFix translateToRosMessage(const GgaFix& fix,
                                               const ros::Time& receipt) const {
    sensor_msgs::NavSatFix message;
    message.header.stamp = receipt;
    message.header.frame_id = frame_id_;
    message.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    message.status.status = fix.quality > 0 ? sensor_msgs::NavSatStatus::STATUS_FIX
                                           : sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    message.latitude = fix.has_latitude ? fix.latitude : std::numeric_limits<double>::quiet_NaN();
    message.longitude = fix.has_longitude ? fix.longitude : std::numeric_limits<double>::quiet_NaN();
    message.altitude = fix.has_altitude_msl && fix.has_geoid_separation
                           ? fix.altitude_msl + fix.geoid_separation
                           : std::numeric_limits<double>::quiet_NaN();
    message.position_covariance.fill(0.0);
    message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    return message;
  }

  std::string frame_id_;
  DiagnosticsReporter* reporter_;
  ros::Publisher publisher_;
};

class ImuBridge {
 public:
  ImuBridge(ros::NodeHandle node, std::string topic, std::string frame_id,
            PacketLayout layout, DiagnosticsReporter* reporter)
      : frame_id_(std::move(frame_id)), layout_(layout), reporter_(reporter) {
    publisher_ = node.advertise<sensor_msgs::Imu>(topic, 1, false);
  }
  void handle(std::vector<Datagram> batch) {
    std::size_t selected = 0, rejected = 0, non_target = 0;
    ImuSample sample;
    const bool found = LatestDatagramSelector::selectNewestDecodable(batch,
          [this](const Datagram& packet) { return ImuParser::decodeStructure(packet, layout_); },
          &sample, &selected, &rejected, &non_target);
    reporter_->recordSelection("imu", rejected, non_target);
    if (found && rejected > 0U)
      reporter_->parseError("imu", "batch rejected=" + std::to_string(rejected), batch.back().ros_receipt, 0U);
    if (found) {
        const Datagram& datagram = batch[selected];
        reporter_->superseded("imu", selected);
        const std::string anomaly = RxValueInspector::imu(sample);
        reporter_->valueAnomaly("imu", anomaly, datagram.ros_receipt);
        publisher_.publish(translateToRosMessage(sample, datagram.ros_receipt));
        reporter_->published("imu", datagram.ros_receipt, datagram.steady_receipt,
                             sample.sec, sample.nsec);
        return;
    }
    reporter_->parseError("imu", "no structurally decodable IMU packet", batch.back().ros_receipt, 0U);
  }
  void reset() {}

 private:
  sensor_msgs::Imu translateToRosMessage(const ImuSample& sample,
                                         const ros::Time& receipt) const {
    sensor_msgs::Imu message;
    message.header.stamp = receipt;
    message.header.frame_id = frame_id_;
    message.orientation.w = sample.orientation_w;
    message.orientation.x = sample.orientation_x;
    message.orientation.y = sample.orientation_y;
    message.orientation.z = sample.orientation_z;
    message.angular_velocity.x = sample.angular_velocity_x;
    message.angular_velocity.y = sample.angular_velocity_y;
    message.angular_velocity.z = sample.angular_velocity_z;
    message.linear_acceleration.x = sample.linear_acceleration_x;
    message.linear_acceleration.y = sample.linear_acceleration_y;
    message.linear_acceleration.z = sample.linear_acceleration_z;
    message.orientation_covariance.fill(0.0);
    message.angular_velocity_covariance.fill(0.0);
    message.linear_acceleration_covariance.fill(0.0);
    message.orientation_covariance[0] = -1.0;
    message.angular_velocity_covariance[0] = -1.0;
    message.linear_acceleration_covariance[0] = -1.0;
    return message;
  }
  std::string frame_id_;
  PacketLayout layout_;
  DiagnosticsReporter* reporter_;
  ros::Publisher publisher_;
};

class StatusBridge {
 public:
  StatusBridge(ros::NodeHandle node, std::string topic, std::string frame_id,
               DiagnosticsReporter* reporter)
      : frame_id_(std::move(frame_id)), reporter_(reporter) {
    publisher_ = node.advertise<interfaces::CompetitionVehicleStatus>(topic, 1, false);
  }
  void handle(std::vector<Datagram> batch) {
    std::size_t selected = 0, rejected = 0, non_target = 0;
    VehicleStatusSample sample;
    const bool found = LatestDatagramSelector::selectNewestDecodable(batch, StatusParser::decodeStructure,
          &sample, &selected, &rejected, &non_target);
    reporter_->recordSelection("status", rejected, non_target);
    if (found && rejected > 0U)
      reporter_->parseError("status", "batch rejected=" + std::to_string(rejected), batch.back().ros_receipt, 0U);
    if (found) {
        const Datagram& datagram = batch[selected];
        reporter_->superseded("status", selected);
        const std::string anomaly = RxValueInspector::status(sample);
        reporter_->valueAnomaly("status", anomaly, datagram.ros_receipt);
        publisher_.publish(translateToRosMessage(sample, datagram.ros_receipt));
        reporter_->published("status", datagram.ros_receipt, datagram.steady_receipt,
                             sample.source_sec, sample.source_nsec);
        return;
    }
    reporter_->parseError("status", "no structurally decodable status packet", batch.back().ros_receipt, 0U);
  }
  void reset() {}

 private:
  interfaces::CompetitionVehicleStatus translateToRosMessage(
      const VehicleStatusSample& s, const ros::Time& receipt) const {
    interfaces::CompetitionVehicleStatus message;
    message.header.stamp = receipt;
    message.header.frame_id = frame_id_;
    message.source_sec = s.source_sec;
    message.source_nsec = s.source_nsec;
    message.control_mode = s.control_mode;
    message.gear = s.gear;
    message.signed_velocity_mps = s.signed_velocity_mps;
    message.map_data_id = s.map_data_id;
    message.accel = s.accel;
    message.brake = s.brake;
    message.size_x_m = s.size_x_m;
    message.size_y_m = s.size_y_m;
    message.size_z_m = s.size_z_m;
    message.overhang_m = s.overhang_m;
    message.wheelbase_m = s.wheelbase_m;
    message.rear_overhang_m = s.rear_overhang_m;
    message.roll_rad = s.roll_rad;
    message.pitch_rad = s.pitch_rad;
    message.yaw_rad = s.yaw_rad;
    message.velocity_x_mps = s.velocity_x_mps;
    message.angular_velocity_x_radps = s.angular_velocity_x_radps;
    message.angular_velocity_y_radps = s.angular_velocity_y_radps;
    message.angular_velocity_z_radps = s.angular_velocity_z_radps;
    message.steering_angle_rad = s.steering_angle_rad;
    message.link_id = s.link_id;
    return message;
  }
  std::string frame_id_;
  DiagnosticsReporter* reporter_;
  ros::Publisher publisher_;
};

std::string describe(const ControlCommand& command) {
  std::ostringstream text;
  text << std::setprecision(17) << "accel=" << command.accel << ",brake=" << command.brake
       << ",steering=" << command.steering;
  return text.str();
}

std::string describe(const LampCommand& command) {
  return "turnSignal=" + std::to_string(command.turn_signal) +
         ",emergencySignal=" + std::to_string(command.emergency_signal);
}

void sendAndRecover(const std::string& channel, UdpSender& sender,
                    const TxSocketRecoveryPolicy& recovery,
                    const std::vector<std::uint8_t>& packet,
                    DiagnosticsReporter& reporter, SteadyClock::time_point started) {
  int error = 0;
  if (sender.send(packet, &error)) {
    reporter.sent(channel, ros::Time::now(), started);
    return;
  }
  reporter.sendError(channel, std::strerror(error), ros::Time::now());
  if (recovery.classify(error, sender.consecutiveErrors()) == SocketAction::kReopen) {
    reporter.socketError(channel, "SOCKET_REOPEN", std::strerror(error), ros::Time::now());
    if (sender.reopen()) {
      reporter.recovered(channel, "SOCKET_RECOVERED", ros::Time::now());
    } else {
      reporter.socketError(channel, "SOCKET_REOPEN", std::strerror(errno), ros::Time::now());
    }
  }
}

class ControlTx {
 public:
  ControlTx(ros::NodeHandle node, const ros::NodeHandle& config, DiagnosticsReporter& reporter)
      : reporter_(reporter),
        freshness_(requiredParam<double>(config, "control/command_timeout_sec"),
                   requiredParam<double>(config, "control/safe_brake_command")),
        sender_(requiredParam<std::string>(config, "control/destination_ip"),
                requiredParam<int>(config, "control/destination_port")),
        recovery_(requiredParam<int>(config, "control/send_error_reopen_threshold")) {
    subscriber_ = node.subscribe<morai_msgs::CtrlCmd>(
        requiredParam<std::string>(config, "control/command_topic"), 1,
        &ControlTx::onCommand, this, ros::TransportHints().tcpNoDelay());
    timer_ = node.createWallTimer(ros::WallDuration(1.0 / requiredParam<double>(config, "control/send_rate_hz")),
                                  &ControlTx::onSendTimer, this);
  }

  void onCommand(const morai_msgs::CtrlCmd::ConstPtr& message) {
    const ros::Time receipt = ros::Time::now();
    const auto steady_receipt = SteadyClock::now();
    const ControlCommand raw{message->accel, message->brake, message->steering};
    const SanitizedControl sanitized = sanitizer_.sanitize(raw);
    latest_.store(sanitized.command, steady_receipt);
    reporter_.commandAccepted(sanitized.changed, receipt, steady_receipt,
                              describe(raw), describe(sanitized.command));
  }

  void onSendTimer(const ros::WallTimerEvent&) {
    const auto started = SteadyClock::now();
    ControlCommand latest;
    double age = 0;
    const bool has_command = latest_.snapshot(&latest, &age);
    bool stale = false;
    const ControlCommand selected = freshness_.select(has_command, latest, age, &stale);
    reporter_.staleFallback("control_tx", age, ros::Time::now(), stale);
    sendAndRecover("control_tx", sender_, recovery_, encoder_.encodeMode1(selected), reporter_, started);
  }

 private:
  DiagnosticsReporter& reporter_;
  ControlCommandSanitizer sanitizer_;
  LatestCommand latest_;
  CommandFreshnessGuard freshness_;
  MoraiControlEncoder encoder_;
  UdpSender sender_;
  TxSocketRecoveryPolicy recovery_;
  ros::Subscriber subscriber_;
  ros::WallTimer timer_;
};

class LampTx {
 public:
  LampTx(ros::NodeHandle node, const ros::NodeHandle& config, DiagnosticsReporter& reporter)
      : reporter_(reporter),
        sender_(requiredParam<std::string>(config, "lamp_control/destination_ip"),
                requiredParam<int>(config, "lamp_control/destination_port")),
        recovery_(requiredParam<int>(config, "lamp_control/send_error_reopen_threshold")) {
    subscriber_ = node.subscribe<morai_msgs::Lamps>(
        requiredParam<std::string>(config, "lamp_control/command_topic"), 1,
        &LampTx::onLamps, this, ros::TransportHints().tcpNoDelay());
  }

  void onLamps(const morai_msgs::Lamps::ConstPtr& message) {
    const auto started = SteadyClock::now();
    const LampCommand raw{message->turnSignal, message->emergencySignal};
    const SanitizedLamp sanitized = sanitizer_.sanitize(raw);
    reporter_.commandAccepted(sanitized.changed, ros::Time::now(), started,
                              describe(raw), describe(sanitized.command), "lamp_tx");
    sendAndRecover("lamp_tx", sender_, recovery_, encoder_.encode(sanitized.command), reporter_, started);
  }

 private:
  DiagnosticsReporter& reporter_;
  LampCommandSanitizer sanitizer_;
  MoraiLampEncoder encoder_;
  UdpSender sender_;
  TxSocketRecoveryPolicy recovery_;
  ros::Subscriber subscriber_;
};

class BridgeNode {
 public:
  BridgeNode() : node_(), private_("~"),
      reporter_(node_, param(private_, "high_hz_diagnostics_topic", std::string("/diagnostics/high_hz"))) {
    setupRx();
    configureChannel("control_tx", "control", "tx", [this]() {
      ChannelConfigValidator::validateTx(private_, "control");
      control_.reset(new ControlTx(node_, private_, reporter_));
    });
    if (param(private_, "lamp_control/enabled", false)) {
      configureChannel("lamp_tx", "lamp_control", "tx", [this]() {
        ChannelConfigValidator::validateTx(private_, "lamp_control");
        lamp_.reset(new LampTx(node_, private_, reporter_));
      });
    }
    stale_timer_ = node_.createWallTimer(ros::WallDuration(0.05), [this](const ros::WallTimerEvent&) {
      reporter_.checkStale();
    });
  }

  void start() {
    for (auto& worker : workers_) worker->start();
  }

  ~BridgeNode() {
    for (auto& worker : workers_) worker->stop();
  }

 private:
  template <typename Setup>
  void configureChannel(const std::string& name, const std::string& prefix,
                         const std::string& direction, Setup setup) {
    const bool rx = direction == "rx";
    const int port = param(private_, prefix + (rx ? "/port" : "/destination_port"), 0);
    reporter_.addChannel(name, direction, port > 0 && port <= 65535 ? port : 0,
        param(private_, prefix + (rx ? "/topic" : "/command_topic"), std::string()),
        rx ? param(private_, prefix + "/stale_timeout", 0.0) : 0.0);
    try {
      setup();
    } catch (const std::bad_alloc&) {
      throw;
    } catch (const std::exception& error) {
      reporter_.configError(name, error.what());
    }
  }

  void addWorker(const std::string& name, int port, const std::string& topic,
                 double stale_timeout, UdpRxWorker::BatchHandler handler,
                 UdpRxWorker::ResetHandler reset) {
    ChannelConfigValidator::validateRx(private_, port, topic, stale_timeout);
    workers_.emplace_back(new UdpRxWorker(
        name, param(private_, "bind_ip", std::string("0.0.0.0")),
        static_cast<std::uint16_t>(port), param(private_, "allowed_source_ip", std::string()),
        static_cast<std::size_t>(param(private_, "receive_buffer_bytes", 4194304)),
        static_cast<unsigned int>(param(private_, "recv_error_reopen_threshold", 3)),
        std::move(handler), std::move(reset), &reporter_));
  }

  void setupRx() {
    const std::string cameras[] = {"front", "left", "right"};
    for (const std::string& camera : cameras) {
      configureChannel("camera_" + camera, "cameras/" + camera, "rx", [this, &camera]() {
      const std::string prefix = "cameras/" + camera + "/";
      const int port = requiredParam<int>(private_, prefix + "port");
      const std::string topic = requiredParam<std::string>(private_, prefix + "topic");
      const std::string info_topic = requiredParam<std::string>(private_, prefix + "camera_info_topic");
      const double stale = requiredParam<double>(private_, prefix + "stale_timeout");
      ChannelConfigValidator::validateRx(private_, port, topic, stale);
      ChannelConfigValidator::validateTopic(info_topic);
        std::unique_ptr<CameraBridge> bridge(new CameraBridge(
            node_, "camera_" + camera, topic, info_topic,
            requiredParam<std::string>(private_, "vehicle_spec/cameras/" + camera + "/frame_id"),
            requiredParam<int>(private_, "vehicle_spec/cameras/" + camera + "/width"),
            requiredParam<int>(private_, "vehicle_spec/cameras/" + camera + "/height"),
            requiredParam<double>(private_, "vehicle_spec/cameras/" + camera + "/horizontal_fov_deg"),
            packetLayoutFromString(requiredParam<std::string>(private_, prefix + "packet_layout")), &reporter_));
        CameraBridge* raw = bridge.get();
        camera_bridges_.push_back(std::move(bridge));
        workers_.emplace_back(new UdpRxWorker(
            "camera_" + camera, param(private_, "bind_ip", std::string("0.0.0.0")), port,
            param(private_, "allowed_source_ip", std::string()),
            param(private_, "receive_buffer_bytes", 4194304),
            param(private_, "recv_error_reopen_threshold", 3),
            [raw](std::vector<Datagram> batch) { raw->handle(std::move(batch)); },
            [raw]() { raw->reset(); }, &reporter_));
      });
    }

    if (param(private_, "gps/enabled", true)) {
      configureChannel("gps", "gps", "rx", [this]() {
      const int port = requiredParam<int>(private_, "gps/port");
      const std::string topic = requiredParam<std::string>(private_, "gps/topic");
      const double stale = requiredParam<double>(private_, "gps/stale_timeout");
      ChannelConfigValidator::validateRx(private_, port, topic, stale);
      ChannelConfigValidator::validateFrame(requiredParam<std::string>(private_, "vehicle_spec/gps_frame_id"));
      gps_.reset(new GpsBridge(node_, topic, requiredParam<std::string>(private_, "vehicle_spec/gps_frame_id"), &reporter_));
      addWorker("gps", port, topic, stale,
                [this](std::vector<Datagram> batch) { gps_->handle(std::move(batch)); },
                [this]() { gps_->reset(); });
      });
    }
    if (param(private_, "imu/enabled", true)) {
      configureChannel("imu", "imu", "rx", [this]() {
      const int port = requiredParam<int>(private_, "imu/port");
      const std::string topic = requiredParam<std::string>(private_, "imu/topic");
      const double stale = requiredParam<double>(private_, "imu/stale_timeout");
      ChannelConfigValidator::validateRx(private_, port, topic, stale);
      ChannelConfigValidator::validateFrame(requiredParam<std::string>(private_, "vehicle_spec/imu_frame_id"));
      imu_.reset(new ImuBridge(node_, topic, requiredParam<std::string>(private_, "vehicle_spec/imu_frame_id"),
                               packetLayoutFromString(requiredParam<std::string>(private_, "imu/packet_layout")), &reporter_));
      addWorker("imu", port, topic, stale,
                [this](std::vector<Datagram> batch) { imu_->handle(std::move(batch)); },
                [this]() { imu_->reset(); });
      });
    }
    if (param(private_, "status/enabled", true)) {
      configureChannel("status", "status", "rx", [this]() {
      const int port = requiredParam<int>(private_, "status/port");
      const std::string topic = requiredParam<std::string>(private_, "status/topic");
      const double stale = requiredParam<double>(private_, "status/stale_timeout");
      ChannelConfigValidator::validateRx(private_, port, topic, stale);
      ChannelConfigValidator::validateFrame(requiredParam<std::string>(private_, "vehicle_spec/base_frame_id"));
      status_.reset(new StatusBridge(node_, topic, requiredParam<std::string>(private_, "vehicle_spec/base_frame_id"), &reporter_));
      addWorker("status", port, topic, stale,
                [this](std::vector<Datagram> batch) { status_->handle(std::move(batch)); },
                [this]() { status_->reset(); });
      });
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_;
  DiagnosticsReporter reporter_;
  std::vector<std::unique_ptr<UdpRxWorker>> workers_;
  std::vector<std::unique_ptr<CameraBridge>> camera_bridges_;
  std::unique_ptr<GpsBridge> gps_;
  std::unique_ptr<ImuBridge> imu_;
  std::unique_ptr<StatusBridge> status_;
  std::unique_ptr<ControlTx> control_;
  std::unique_ptr<LampTx> lamp_;
  ros::WallTimer stale_timer_;
};

}  // namespace
}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "morai_udp_bridge");
  try {
    morai_udp_bridge::BridgeNode bridge;
    bridge.start();
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
  } catch (const std::bad_alloc&) {
    throw;
  } catch (const std::exception& error) {
    ROS_FATAL("morai_udp_bridge startup failed: %s", error.what());
    return 1;
  }
}
