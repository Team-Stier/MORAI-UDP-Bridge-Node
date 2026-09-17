#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
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
};

class RxValueInspector {
 public:
  static std::string camera(const CameraPacket& packet) {
    if (packet.declared_size == 0U) return "camera declared size is zero";
    if (packet.has_timestamp && packet.nsec >= 1000000000U) return "camera nsec is outside range";
    return {};
  }
  static std::string gps(const GgaFix& fix) {
    if (fix.quality < 0 || fix.satellites < 0) return "GGA quality or satellites is negative";
    if ((fix.has_latitude && !std::isfinite(fix.latitude)) ||
        (fix.has_longitude && !std::isfinite(fix.longitude))) return "GGA coordinate is non-finite";
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
    if (sample.source_nsec < 0 || sample.source_nsec >= 1000000000)
      return "status nsec is outside range";
    if (sample.control_mode < 1U || sample.control_mode > 2U) return "status control_mode is unknown";
    if (sample.gear > 5U) return "status gear is unknown";
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
    if (frame_id_.empty()) throw std::runtime_error("camera frame_id is empty");
    image_publisher_ = node.advertise<sensor_msgs::CompressedImage>(topic, 1, false);
    info_publisher_ = node.advertise<sensor_msgs::CameraInfo>(info_topic, 1, false);
  }

  void handle(std::vector<Datagram> batch) {
    for (const Datagram& datagram : batch) {
      try {
        CameraPacket packet = decodeCameraPacket(datagram.bytes.data(), datagram.bytes.size(), layout_);
        const std::string anomaly = RxValueInspector::camera(packet);
        if (!anomaly.empty()) reporter_->valueAnomaly(name_, anomaly, datagram.ros_receipt);
        std::string drop_reason;
        CameraFrame frame;
        if (assembler_.add(std::move(packet), datagram.steady_receipt, &frame, &drop_reason)) {
          publish(translateToRosMessage(frame, datagram.ros_receipt));
          reporter_->published(name_, datagram.ros_receipt, frame.first_receipt,
                               frame.source_sec, frame.source_nsec);
        } else if (!drop_reason.empty()) {
          reporter_->dropped(name_, drop_reason, datagram.ros_receipt);
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
    for (std::size_t i = batch.size(); i > 0U; --i) {
      const Datagram& datagram = batch[i - 1U];
      try {
        const GgaFix fix = decodeGgaDatagram(datagram.bytes.data(), datagram.bytes.size());
        reporter_->superseded("gps", i - 1U);
        const std::string anomaly = RxValueInspector::gps(fix);
        if (!anomaly.empty()) reporter_->valueAnomaly("gps", anomaly, datagram.ros_receipt);
        publisher_.publish(translateToRosMessage(fix, datagram.ros_receipt));
        reporter_->setExtra("gps", "gga_utc", fix.utc);
        reporter_->setExtra("gps", "quality", std::to_string(fix.quality));
        reporter_->setExtra("gps", "satellites", std::to_string(fix.satellites));
        reporter_->setExtra("gps", "hdop", fix.has_hdop ? std::to_string(fix.hdop) : "nan");
        reporter_->published("gps", datagram.ros_receipt, datagram.steady_receipt);
        return;
      } catch (const NonTargetPacket&) {
        ++non_target;
      } catch (const ProtocolError&) {
        ++rejected;
      }
    }
    const ros::Time stamp = batch.back().ros_receipt;
    if (non_target == batch.size()) reporter_->ignored("gps", "NON_GGA", stamp);
    else reporter_->parseError("gps", "no structurally decodable GGA; rejected=" + std::to_string(rejected), stamp);
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
    message.altitude = fix.has_altitude_msl
                           ? fix.altitude_msl + (fix.has_geoid_separation ? fix.geoid_separation : 0.0)
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
    for (std::size_t i = batch.size(); i > 0U; --i) {
      const Datagram& datagram = batch[i - 1U];
      try {
        const ImuSample sample = decodeImuPacket(datagram.bytes.data(), datagram.bytes.size(), layout_);
        reporter_->superseded("imu", i - 1U);
        const std::string anomaly = RxValueInspector::imu(sample);
        if (!anomaly.empty()) reporter_->valueAnomaly("imu", anomaly, datagram.ros_receipt);
        publisher_.publish(translateToRosMessage(sample, datagram.ros_receipt));
        reporter_->published("imu", datagram.ros_receipt, datagram.steady_receipt,
                             sample.sec, sample.nsec);
        return;
      } catch (const ProtocolError&) {
      }
    }
    reporter_->parseError("imu", "no structurally decodable IMU packet", batch.back().ros_receipt);
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
    for (std::size_t i = batch.size(); i > 0U; --i) {
      const Datagram& datagram = batch[i - 1U];
      try {
        const VehicleStatusSample sample = decodeVehicleStatusPacket(datagram.bytes.data(), datagram.bytes.size());
        reporter_->superseded("status", i - 1U);
        const std::string anomaly = RxValueInspector::status(sample);
        if (!anomaly.empty()) reporter_->valueAnomaly("status", anomaly, datagram.ros_receipt);
        publisher_.publish(translateToRosMessage(sample, datagram.ros_receipt));
        reporter_->published("status", datagram.ros_receipt, datagram.steady_receipt,
                             sample.source_sec, sample.source_nsec);
        return;
      } catch (const ProtocolError&) {
      }
    }
    reporter_->parseError("status", "no structurally decodable status packet", batch.back().ros_receipt);
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

class BridgeNode {
 public:
  BridgeNode() : node_(), private_("~"),
      reporter_(node_, param(private_, "diagnostics_topic", std::string("/diagnostics")),
                param(private_, "high_hz_diagnostics_topic", std::string("/diagnostics/high_hz")),
                param(private_, "diagnostics_period", 1.0)) {
    setupControl();
    setupRx();
    setupLamp();
    stale_timer_ = node_.createWallTimer(ros::WallDuration(0.05), [this](const ros::WallTimerEvent&) {
      reporter_.checkStale();
    });
  }

  ~BridgeNode() {
    for (auto& worker : workers_) worker->stop();
  }

  void start() {
    for (auto& worker : workers_) worker->start();
  }

 private:
  void addWorker(const std::string& name, int port, const std::string& topic,
                 double stale_timeout, UdpRxWorker::BatchHandler handler,
                 UdpRxWorker::ResetHandler reset) {
    ChannelConfigValidator::validatePort(port);
    reporter_.addChannel(name, "rx", static_cast<std::uint16_t>(port), topic, stale_timeout);
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
      const std::string prefix = "cameras/" + camera + "/";
      const int port = requiredParam<int>(private_, prefix + "port");
      const std::string topic = requiredParam<std::string>(private_, prefix + "topic");
      const std::string info_topic = requiredParam<std::string>(private_, prefix + "camera_info_topic");
      const double stale = requiredParam<double>(private_, prefix + "stale_timeout");
      reporter_.addChannel("camera_" + camera, "rx", port, topic, stale);
      try {
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
      } catch (const std::exception& error) {
        reporter_.configError("camera_" + camera, error.what());
      }
    }

    if (param(private_, "gps/enabled", true)) {
      const int port = requiredParam<int>(private_, "gps/port");
      const std::string topic = requiredParam<std::string>(private_, "gps/topic");
      const double stale = requiredParam<double>(private_, "gps/stale_timeout");
      gps_.reset(new GpsBridge(node_, topic, requiredParam<std::string>(private_, "vehicle_spec/gps_frame_id"), &reporter_));
      addWorker("gps", port, topic, stale,
                [this](std::vector<Datagram> batch) { gps_->handle(std::move(batch)); },
                [this]() { gps_->reset(); });
    }
    if (param(private_, "imu/enabled", true)) {
      const int port = requiredParam<int>(private_, "imu/port");
      const std::string topic = requiredParam<std::string>(private_, "imu/topic");
      const double stale = requiredParam<double>(private_, "imu/stale_timeout");
      imu_.reset(new ImuBridge(node_, topic, requiredParam<std::string>(private_, "vehicle_spec/imu_frame_id"),
                               packetLayoutFromString(requiredParam<std::string>(private_, "imu/packet_layout")), &reporter_));
      addWorker("imu", port, topic, stale,
                [this](std::vector<Datagram> batch) { imu_->handle(std::move(batch)); },
                [this]() { imu_->reset(); });
    }
    if (param(private_, "status/enabled", true)) {
      const int port = requiredParam<int>(private_, "status/port");
      const std::string topic = requiredParam<std::string>(private_, "status/topic");
      const double stale = requiredParam<double>(private_, "status/stale_timeout");
      ChannelConfigValidator::validatePort(port);
      reporter_.addChannel("status", "rx", static_cast<std::uint16_t>(port), topic, stale);
      reporter_.setSocketBuffer("status",
                                param(private_, "receive_buffer_bytes", 4194304),
                                control_socket_->actualReceiveBuffer());
      status_.reset(new StatusBridge(node_, topic, requiredParam<std::string>(private_, "vehicle_spec/base_frame_id"), &reporter_));
      workers_.emplace_back(new CompetitionRxWorker(
          "status", param(private_, "allowed_source_ip", std::string()),
          param(private_, "recv_error_reopen_threshold", 3),
          [this](std::vector<Datagram> batch) { status_->handle(std::move(batch)); },
          [this]() { status_->reset(); }, &reporter_, control_socket_.get()));
    }
  }

  void setupControl() {
    const int destination_port = requiredParam<int>(private_, "control/destination_port");
    const int source_port = requiredParam<int>(private_, "status/port");
    ChannelConfigValidator::validatePort(destination_port);
    ChannelConfigValidator::validatePort(source_port);
    const double rate = requiredParam<double>(private_, "control/send_rate_hz");
    ChannelConfigValidator::validatePositive(rate, "control send_rate_hz");
    reporter_.addChannel("control_tx", "tx", destination_port,
                         requiredParam<std::string>(private_, "control/command_topic"), 0.0);
    control_socket_.reset(new ControlUdpSocket(
        requiredParam<std::string>(private_, "control/destination_ip"),
        static_cast<std::uint16_t>(destination_port),
        param(private_, "bind_ip", std::string("0.0.0.0")),
        static_cast<std::uint16_t>(source_port),
        param(private_, "receive_buffer_bytes", 4194304),
        requiredParam<int>(private_, "control/send_error_reopen_threshold")));
    freshness_.reset(new CommandFreshnessGuard(
        requiredParam<double>(private_, "control/command_timeout_sec"),
        requiredParam<double>(private_, "control/safe_brake_command")));
    control_subscriber_ = node_.subscribe<morai_msgs::CtrlCmd>(
        requiredParam<std::string>(private_, "control/command_topic"), 1,
        &BridgeNode::onCommand, this, ros::TransportHints().tcpNoDelay());
    send_timer_ = node_.createWallTimer(ros::WallDuration(1.0 / rate), &BridgeNode::onSendTimer, this);
  }

  void setupLamp() {
    if (!param(private_, "lamp_control/enabled", false)) return;
    const int port = requiredParam<int>(private_, "lamp_control/destination_port");
    ChannelConfigValidator::validatePort(port);
    reporter_.addChannel("lamp_tx", "tx", port,
                         requiredParam<std::string>(private_, "lamp_control/command_topic"), 0.0);
    lamp_sender_.reset(new UdpSender(requiredParam<std::string>(private_, "lamp_control/destination_ip"), port,
                                     requiredParam<int>(private_, "lamp_control/send_error_reopen_threshold")));
    lamp_subscriber_ = node_.subscribe<morai_msgs::Lamps>(
        requiredParam<std::string>(private_, "lamp_control/command_topic"), 1,
        &BridgeNode::onLamps, this, ros::TransportHints().tcpNoDelay());
  }

  void onCommand(const morai_msgs::CtrlCmd::ConstPtr& message) {
    const ros::Time receipt = ros::Time::now();
    ControlCommand raw;
    raw.accel = message->accel;
    raw.brake = message->brake;
    raw.steering = message->steering;
    const SanitizedControl sanitized = sanitizer_.sanitize(raw);
    if (sanitized.changed) ROS_WARN_THROTTLE(1.0, "control command sanitized: %s", sanitized.reason.c_str());
    latest_.store(sanitized.command, SteadyClock::now());
    reporter_.commandAccepted(sanitized.changed, receipt);
  }

  void onSendTimer(const ros::WallTimerEvent&) {
    const auto started = SteadyClock::now();
    ControlCommand latest;
    double age = 0.0;
    const bool has_command = latest_.snapshot(&latest, &age);
    bool stale = false;
    const ControlCommand selected = freshness_->select(has_command, latest, age, &stale);
    if (stale) reporter_.staleFallback("control_tx", age, ros::Time::now());
    else reporter_.recovered("control_tx", "STALE_COMMAND_RECOVERED", ros::Time::now());
    const std::vector<std::uint8_t> packet = control_encoder_.encodeMode1(selected);
    int error = 0;
    if (control_socket_->send(packet, &error)) reporter_.sent("control_tx", ros::Time::now(), started);
    else reporter_.sendError("control_tx", std::strerror(error), ros::Time::now());
  }

  void onLamps(const morai_msgs::Lamps::ConstPtr& message) {
    const auto started = SteadyClock::now();
    LampCommand raw;
    raw.turn_signal = message->turnSignal;
    raw.emergency_signal = message->emergencySignal;
    const SanitizedLamp sanitized = lamp_sanitizer_.sanitize(raw);
    if (sanitized.changed) reporter_.valueAnomaly("lamp_tx", "lamp value replaced with off", ros::Time::now());
    int error = 0;
    if (lamp_sender_->send(lamp_encoder_.encode(sanitized.command), &error))
      reporter_.sent("lamp_tx", ros::Time::now(), started);
    else reporter_.sendError("lamp_tx", std::strerror(error), ros::Time::now());
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_;
  DiagnosticsReporter reporter_;
  std::vector<std::unique_ptr<UdpRxWorkerBase>> workers_;
  std::vector<std::unique_ptr<CameraBridge>> camera_bridges_;
  std::unique_ptr<GpsBridge> gps_;
  std::unique_ptr<ImuBridge> imu_;
  std::unique_ptr<StatusBridge> status_;
  ControlCommandSanitizer sanitizer_;
  MoraiControlEncoder control_encoder_;
  LatestCommand latest_;
  std::unique_ptr<CommandFreshnessGuard> freshness_;
  std::unique_ptr<ControlUdpSocket> control_socket_;
  LampCommandSanitizer lamp_sanitizer_;
  MoraiLampEncoder lamp_encoder_;
  std::unique_ptr<UdpSender> lamp_sender_;
  ros::Subscriber control_subscriber_;
  ros::Subscriber lamp_subscriber_;
  ros::WallTimer send_timer_;
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
