#include "morai_udp_bridge/protocol.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

namespace morai_udp_bridge {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::uint32_t readU32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) |
         (static_cast<std::uint32_t>(p[1]) << 8U) |
         (static_cast<std::uint32_t>(p[2]) << 16U) |
         (static_cast<std::uint32_t>(p[3]) << 24U);
}

std::int32_t readI32(const std::uint8_t* p) {
  return static_cast<std::int32_t>(readU32(p));
}

std::uint64_t readU64(const std::uint8_t* p) {
  std::uint64_t value = 0U;
  for (std::size_t i = 0; i < 8U; ++i) value |= static_cast<std::uint64_t>(p[i]) << (i * 8U);
  return value;
}

float readF32(const std::uint8_t* p) {
  const std::uint32_t bits = readU32(p);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double readF64(const std::uint8_t* p) {
  const std::uint64_t bits = readU64(p);
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void writeU32(std::vector<std::uint8_t>* out, std::size_t at, std::uint32_t value) {
  for (std::size_t i = 0; i < 4U; ++i) (*out)[at + i] = static_cast<std::uint8_t>(value >> (i * 8U));
}

void writeF32(std::vector<std::uint8_t>* out, std::size_t at, float value) {
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  writeU32(out, at, bits);
}

bool startsWith(const std::uint8_t* data, std::size_t size, const char* value,
                std::size_t length) {
  return size >= length && std::equal(value, value + length, data);
}

bool jpegSoi(const std::vector<std::uint8_t>& bytes) {
  return bytes.size() >= 2U && bytes[0] == 0xffU && bytes[1] == 0xd8U;
}

std::string trim(const std::string& value) {
  const std::size_t first = value.find_first_not_of("\r\n ");
  if (first == std::string::npos) return {};
  return value.substr(first, value.find_last_not_of("\r\n ") - first + 1U);
}

std::vector<std::string> split(const std::string& value, char delimiter) {
  std::vector<std::string> result;
  std::size_t begin = 0;
  while (true) {
    const std::size_t end = value.find(delimiter, begin);
    result.push_back(value.substr(begin, end - begin));
    if (end == std::string::npos) return result;
    begin = end + 1U;
  }
}

double parseDouble(const std::string& value, const char* field) {
  if (value.empty()) throw ProtocolError(std::string("empty GGA ") + field);
  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (errno == ERANGE || end == value.c_str() || *end != '\0')
    throw ProtocolError(std::string("malformed GGA ") + field);
  return parsed;
}

long parseLong(const std::string& value, const char* field) {
  if (value.empty()) return 0;
  errno = 0;
  char* end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (errno == ERANGE || end == value.c_str() || *end != '\0')
    throw ProtocolError(std::string("malformed GGA ") + field);
  return parsed;
}

bool optionalDouble(const std::string& value, const char* field, double* output) {
  if (value.empty()) return false;
  *output = parseDouble(value, field);
  return true;
}

std::string checkedNmeaBody(const std::string& raw) {
  const std::string sentence = trim(raw);
  if (sentence.empty() || sentence.front() != '$') throw ProtocolError("NMEA missing $");
  const std::size_t star = sentence.rfind('*');
  if (star == std::string::npos || sentence.size() != star + 3U)
    throw ProtocolError("NMEA checksum field malformed");
  unsigned int expected = 0;
  std::istringstream input(sentence.substr(star + 1U));
  input >> std::hex >> expected;
  if (!input || !input.eof() || expected > 0xffU) throw ProtocolError("NMEA checksum malformed");
  std::uint8_t actual = 0U;
  for (std::size_t i = 1U; i < star; ++i) actual ^= static_cast<std::uint8_t>(sentence[i]);
  if (actual != expected) throw ProtocolError("NMEA checksum mismatch");
  return sentence.substr(1U, star - 1U);
}

double coordinate(const std::string& value, const std::string& direction,
                  bool latitude) {
  if (value.empty()) throw ProtocolError("GGA coordinate empty");
  if ((latitude && direction != "N" && direction != "S") ||
      (!latitude && direction != "E" && direction != "W"))
    throw ProtocolError("GGA coordinate direction malformed");
  const double raw = parseDouble(value, "coordinate");
  const int degrees = static_cast<int>(std::floor(raw / 100.0));
  const double minutes = raw - degrees * 100.0;
  double result = degrees + minutes / 60.0;
  if (raw < 0.0 || minutes >= 60.0 || result > (latitude ? 90.0 : 180.0))
    throw ProtocolError("GGA coordinate out of range");
  if (direction == "S" || direction == "W") result = -result;
  return result;
}

double finiteOrZero(double value, bool* changed) {
  if (std::isfinite(value)) return value;
  *changed = true;
  return 0.0;
}

double clamp(double value, double low, double high, bool* changed) {
  const double result = std::max(low, std::min(high, value));
  if (result != value) *changed = true;
  return result;
}

}  // namespace

PacketLayout packetLayoutFromString(const std::string& value) {
  if (value == "legacy") return PacketLayout::kLegacy;
  if (value == "timestamped") return PacketLayout::kTimestamped;
  throw ProtocolError("packet_layout must be legacy or timestamped");
}

CameraPacket decodeCameraPacket(const std::uint8_t* data, std::size_t size,
                                PacketLayout layout) {
  const std::size_t payload_offset = layout == PacketLayout::kTimestamped ? 19U : 11U;
  if (!data || size < payload_offset + 2U || !startsWith(data, size, "MOR", 3U))
    throw ProtocolError("camera header or length malformed");
  if (data[size - 1U] != 'I' || (data[size - 2U] != 'A' && data[size - 2U] != 'E'))
    throw ProtocolError("camera tail malformed");
  CameraPacket packet;
  if (layout == PacketLayout::kTimestamped) {
    packet.has_timestamp = true;
    packet.sec = readU32(data + 3U);
    packet.nsec = readU32(data + 7U);
    packet.index = readU32(data + 11U);
    packet.declared_size = readU32(data + 15U);
  } else {
    packet.index = readU32(data + 3U);
    packet.declared_size = readU32(data + 7U);
  }
  packet.final = data[size - 2U] == 'E';
  packet.payload.assign(data + payload_offset, data + size - 2U);
  return packet;
}

CameraFrameAssembler::CameraFrameAssembler(std::size_t max_chunks,
                                           std::chrono::milliseconds timeout)
    : max_chunks_(max_chunks), timeout_(timeout) {
  if (max_chunks_ == 0U || timeout_.count() <= 0) throw std::invalid_argument("invalid camera limits");
}

void CameraFrameAssembler::reset() {
  active_ = false;
  has_key_ = false;
  has_start_ = false;
  has_final_ = false;
  chunks_.clear();
}

void CameraFrameAssembler::start(const CameraPacket& packet,
                                 std::chrono::steady_clock::time_point receipt) {
  active_ = true;
  has_key_ = packet.has_timestamp;
  key_sec_ = packet.sec;
  key_nsec_ = packet.nsec;
  first_receipt_ = receipt;
}

bool CameraFrameAssembler::add(CameraPacket packet,
                               std::chrono::steady_clock::time_point receipt,
                               CameraFrame* frame, std::string* drop_reason) {
  if (!frame || !drop_reason) throw std::invalid_argument("camera output is null");
  drop_reason->clear();
  if (active_ && receipt - first_receipt_ > timeout_) {
    *drop_reason = "ASSEMBLY_TIMEOUT";
    reset();
  }
  if (active_ && packet.has_timestamp && has_key_ &&
      (packet.sec != key_sec_ || packet.nsec != key_nsec_)) {
    *drop_reason = "NEW_FRAME";
    reset();
  }
  if (!active_) start(packet, receipt);
  if (packet.index >= max_chunks_) {
    *drop_reason = "MAX_CHUNKS";
    reset();
    return false;
  }
  if (packet.index == 0U || (packet.index == 1U && jpegSoi(packet.payload))) {
    if (!has_start_) {
      start_index_ = packet.index;
      has_start_ = true;
    }
  }
  chunks_[packet.index] = std::move(packet.payload);
  if (packet.final) {
    final_index_ = packet.index;
    has_final_ = true;
  }
  if (!has_start_ || !has_final_ || final_index_ < start_index_) return false;
  std::vector<std::uint8_t> jpeg;
  for (std::uint32_t i = start_index_;; ++i) {
    const auto found = chunks_.find(i);
    if (found == chunks_.end()) return false;
    jpeg.insert(jpeg.end(), found->second.begin(), found->second.end());
    if (i == final_index_) break;
  }
  const std::array<std::uint8_t, 2> eoi{{0xffU, 0xd9U}};
  const auto end = jpeg.size() >= 2U ? std::search(jpeg.begin() + 2U, jpeg.end(), eoi.begin(), eoi.end()) : jpeg.end();
  if (!jpegSoi(jpeg) || end == jpeg.end()) {
    *drop_reason = "JPEG_STRUCTURE";
    reset();
    return false;
  }
  frame->jpeg.assign(jpeg.begin(), end + 2U);
  frame->source_sec = key_sec_;
  frame->source_nsec = key_nsec_;
  frame->first_receipt = first_receipt_;
  reset();
  return true;
}

GgaFix decodeGgaDatagram(const std::uint8_t* data, std::size_t size) {
  if (!data) throw ProtocolError("GPS data is null");
  std::string text;
  text.reserve(size);
  for (std::size_t i = 0; i < size; ++i) {
    if (data[i] > 0x7fU) throw ProtocolError("GPS is not ASCII");
    if (data[i] != 0U) text.push_back(static_cast<char>(data[i]));
  }
  for (std::size_t begin = text.find('$'); begin != std::string::npos;) {
    const std::size_t next = text.find('$', begin + 1U);
    const std::string sentence = trim(text.substr(begin, next - begin));
    const std::size_t comma = sentence.find(',');
    if (comma >= 3U && sentence.substr(comma - 3U, 3U) == "GGA") {
      const std::vector<std::string> fields = split(checkedNmeaBody(sentence), ',');
      if (fields.size() < 15U) throw ProtocolError("GGA field count malformed");
      GgaFix fix;
      fix.utc = fields[1];
      fix.quality = static_cast<int>(parseLong(fields[6], "quality"));
      fix.satellites = static_cast<int>(parseLong(fields[7], "satellites"));
      if (!fields[2].empty() || !fields[3].empty()) {
        fix.latitude = coordinate(fields[2], fields[3], true);
        fix.has_latitude = true;
      }
      if (!fields[4].empty() || !fields[5].empty()) {
        fix.longitude = coordinate(fields[4], fields[5], false);
        fix.has_longitude = true;
      }
      fix.has_hdop = optionalDouble(fields[8], "HDOP", &fix.hdop);
      fix.has_altitude_msl = optionalDouble(fields[9], "altitude", &fix.altitude_msl);
      fix.has_geoid_separation = optionalDouble(fields[11], "geoid", &fix.geoid_separation);
      return fix;
    }
    begin = next;
  }
  throw NonTargetPacket("GPS datagram has no GGA");
}

ImuSample decodeImuPacket(const std::uint8_t* data, std::size_t size,
                          PacketLayout layout) {
  const std::size_t expected = layout == PacketLayout::kTimestamped ? 115U : 107U;
  const std::size_t values_at = layout == PacketLayout::kTimestamped ? 33U : 25U;
  if (!data || size != expected || !startsWith(data, size, "#IMUData$", 9U) ||
      data[size - 2U] != '\r' || data[size - 1U] != '\n')
    throw ProtocolError("IMU envelope malformed");
  ImuSample sample;
  if (layout == PacketLayout::kTimestamped) {
    sample.has_timestamp = true;
    sample.sec = readU32(data + 25U);
    sample.nsec = readU32(data + 29U);
  }
  double* fields[] = {&sample.orientation_w, &sample.orientation_x, &sample.orientation_y,
                      &sample.orientation_z, &sample.angular_velocity_x,
                      &sample.angular_velocity_y, &sample.angular_velocity_z,
                      &sample.linear_acceleration_x, &sample.linear_acceleration_y,
                      &sample.linear_acceleration_z};
  for (std::size_t i = 0; i < 10U; ++i) *fields[i] = readF64(data + values_at + i * 8U);
  return sample;
}

VehicleStatusSample decodeVehicleStatusPacket(const std::uint8_t* data,
                                               std::size_t size) {
  if (!data || size != 245U || !startsWith(data, size, "#MoraiInfo$", 11U) ||
      readU32(data + 11U) != 216U || data[243U] != '\r' || data[244U] != '\n')
    throw ProtocolError("competition status envelope malformed");
  const std::uint8_t* p = data + 27U;
  VehicleStatusSample s;
  s.source_sec = readF32(p);
  s.source_nsec = readF32(p + 4U);
  s.control_mode = p[8U];
  s.gear = p[9U];
  s.signed_velocity_mps = readF32(p + 10U) / 3.6F;
  s.map_data_id = readI32(p + 14U);
  s.accel = readF32(p + 18U);
  s.brake = readF32(p + 22U);
  s.size_x_m = readF32(p + 26U);
  s.size_y_m = readF32(p + 30U);
  s.size_z_m = readF32(p + 34U);
  s.overhang_m = readF32(p + 38U);
  s.wheelbase_m = readF32(p + 42U);
  s.rear_overhang_m = readF32(p + 46U);
  const float deg = static_cast<float>(kPi / 180.0);
  s.roll_rad = readF32(p + 62U) * deg;
  s.pitch_rad = readF32(p + 66U) * deg;
  s.yaw_rad = readF32(p + 70U) * deg;
  s.velocity_x_mps = readF32(p + 74U) / 3.6F;
  s.angular_velocity_x_radps = readF32(p + 86U) * deg;
  s.angular_velocity_y_radps = readF32(p + 90U) * deg;
  s.angular_velocity_z_radps = readF32(p + 94U) * deg;
  s.steering_angle_rad = readF32(p + 110U) * deg;
  const char* link = reinterpret_cast<const char*>(p + 118U);
  s.link_id.assign(link, std::find(link, link + 38U, '\0'));
  return s;
}

SanitizedControl ControlCommandSanitizer::sanitize(const ControlCommand& input) const {
  SanitizedControl result;
  result.command.accel = clamp(finiteOrZero(input.accel, &result.changed), 0.0, 1.0, &result.changed);
  result.command.brake = clamp(finiteOrZero(input.brake, &result.changed), 0.0, 1.0, &result.changed);
  result.command.steering = clamp(finiteOrZero(input.steering, &result.changed), -1.0, 1.0, &result.changed);
  if (result.command.accel > 0.0 && result.command.brake > 0.0) {
    result.command.accel = 0.0;
    result.changed = true;
  }
  if (result.changed) result.reason = "non-finite->0, clamp, or brake-wins";
  return result;
}

std::vector<std::uint8_t> MoraiControlEncoder::encodeMode1(
    const ControlCommand& command) const {
  std::vector<std::uint8_t> packet(55U, 0U);
  std::copy_n("#MoraiCtrlCmd$", 14U, packet.begin());
  writeU32(&packet, 14U, 23U);
  packet[30U] = 2U;
  packet[31U] = 4U;
  packet[32U] = 1U;
  writeF32(&packet, 33U, 0.0F);
  writeF32(&packet, 37U, 0.0F);
  writeF32(&packet, 41U, static_cast<float>(command.accel));
  writeF32(&packet, 45U, static_cast<float>(command.brake));
  writeF32(&packet, 49U, static_cast<float>(command.steering));
  packet[53U] = '\r';
  packet[54U] = '\n';
  return packet;
}

SanitizedLamp LampCommandSanitizer::sanitize(const LampCommand& input) const {
  SanitizedLamp result;
  result.command = input;
  if (result.command.turn_signal < 0 || result.command.turn_signal > 2) {
    result.command.turn_signal = 0;
    result.changed = true;
  }
  if (result.command.emergency_signal < 0 || result.command.emergency_signal > 1) {
    result.command.emergency_signal = 0;
    result.changed = true;
  }
  return result;
}

std::vector<std::uint8_t> MoraiLampEncoder::encode(const LampCommand& command) const {
  std::vector<std::uint8_t> packet(33U, 0U);
  std::copy_n("#LampControl$", 13U, packet.begin());
  writeU32(&packet, 13U, 2U);
  packet[29U] = static_cast<std::uint8_t>(command.turn_signal);
  packet[30U] = static_cast<std::uint8_t>(command.emergency_signal);
  packet[31U] = '\r';
  packet[32U] = '\n';
  return packet;
}

}  // namespace morai_udp_bridge
