#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace morai_udp_bridge {

class ProtocolError : public std::runtime_error {
 public:
  explicit ProtocolError(const std::string& message) : std::runtime_error(message) {}
};

class NonTargetPacket : public ProtocolError {
 public:
  explicit NonTargetPacket(const std::string& message) : ProtocolError(message) {}
};

enum class PacketLayout { kLegacy, kTimestamped };
PacketLayout packetLayoutFromString(const std::string& value);

struct CameraPacket {
  std::uint32_t index{0};
  std::uint32_t declared_size{0};
  std::vector<std::uint8_t> payload;
  bool final{false};
  bool has_timestamp{false};
  std::uint32_t sec{0};
  std::uint32_t nsec{0};
};

struct CameraFrame {
  std::vector<std::uint8_t> jpeg;
  std::uint32_t source_sec{0};
  std::uint32_t source_nsec{0};
  std::chrono::steady_clock::time_point first_receipt;
};

CameraPacket decodeCameraPacket(const std::uint8_t* data, std::size_t size,
                                PacketLayout layout);

class CameraFrameAssembler {
 public:
  CameraFrameAssembler(std::size_t max_chunks,
                       std::chrono::milliseconds timeout);
  bool add(CameraPacket packet, std::chrono::steady_clock::time_point receipt,
           CameraFrame* frame, std::string* drop_reason);
  void reset();

 private:
  void start(const CameraPacket& packet,
             std::chrono::steady_clock::time_point receipt);
  bool active_{false};
  bool has_key_{false};
  std::uint32_t key_sec_{0};
  std::uint32_t key_nsec_{0};
  std::uint32_t start_index_{0};
  bool has_start_{false};
  std::uint32_t final_index_{0};
  bool has_final_{false};
  std::size_t max_chunks_;
  std::chrono::milliseconds timeout_;
  std::chrono::steady_clock::time_point first_receipt_;
  std::map<std::uint32_t, std::vector<std::uint8_t>> chunks_;
};

struct GgaFix {
  std::string utc;
  bool has_latitude{false};
  bool has_longitude{false};
  double latitude{0.0};
  double longitude{0.0};
  int quality{0};
  int satellites{0};
  bool has_hdop{false};
  double hdop{0.0};
  bool has_altitude_msl{false};
  double altitude_msl{0.0};
  bool has_geoid_separation{false};
  double geoid_separation{0.0};
};
GgaFix decodeGgaDatagram(const std::uint8_t* data, std::size_t size);

struct ImuSample {
  double orientation_w{0.0};
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double angular_velocity_x{0.0};
  double angular_velocity_y{0.0};
  double angular_velocity_z{0.0};
  double linear_acceleration_x{0.0};
  double linear_acceleration_y{0.0};
  double linear_acceleration_z{0.0};
  bool has_timestamp{false};
  std::uint32_t sec{0};
  std::uint32_t nsec{0};
};
ImuSample decodeImuPacket(const std::uint8_t* data, std::size_t size,
                          PacketLayout layout);

struct VehicleStatusSample {
  float source_sec{0.0F};
  float source_nsec{0.0F};
  std::uint8_t control_mode{0};
  std::uint8_t gear{0};
  float signed_velocity_mps{0.0F};
  std::int32_t map_data_id{0};
  float accel{0.0F};
  float brake{0.0F};
  float size_x_m{0.0F};
  float size_y_m{0.0F};
  float size_z_m{0.0F};
  float overhang_m{0.0F};
  float wheelbase_m{0.0F};
  float rear_overhang_m{0.0F};
  float roll_rad{0.0F};
  float pitch_rad{0.0F};
  float yaw_rad{0.0F};
  float velocity_x_mps{0.0F};
  float angular_velocity_x_radps{0.0F};
  float angular_velocity_y_radps{0.0F};
  float angular_velocity_z_radps{0.0F};
  float steering_angle_rad{0.0F};
  std::string link_id;
};
VehicleStatusSample decodeVehicleStatusPacket(const std::uint8_t* data,
                                               std::size_t size);

struct ControlCommand {
  double accel{0.0};
  double brake{0.0};
  double steering{0.0};
};

struct SanitizedControl {
  ControlCommand command;
  bool changed{false};
  std::string reason;
};

class ControlCommandSanitizer {
 public:
  SanitizedControl sanitize(const ControlCommand& input) const;
};

class MoraiControlEncoder {
 public:
  std::vector<std::uint8_t> encodeMode1(const ControlCommand& command) const;
};

struct LampCommand {
  std::int8_t turn_signal{0};
  std::int8_t emergency_signal{0};
};

struct SanitizedLamp {
  LampCommand command;
  bool changed{false};
};

class LampCommandSanitizer {
 public:
  SanitizedLamp sanitize(const LampCommand& input) const;
};

class MoraiLampEncoder {
 public:
  std::vector<std::uint8_t> encode(const LampCommand& command) const;
};

}  // namespace morai_udp_bridge
