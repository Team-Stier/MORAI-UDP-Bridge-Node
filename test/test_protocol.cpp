#include <cmath>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "morai_udp_bridge/protocol.hpp"

namespace morai_udp_bridge {
namespace {

std::uint32_t readU32(const std::vector<std::uint8_t>& bytes, std::size_t at) {
  return bytes[at] | (bytes[at + 1U] << 8U) | (bytes[at + 2U] << 16U) |
         (bytes[at + 3U] << 24U);
}

float readF32(const std::vector<std::uint8_t>& bytes, std::size_t at) {
  const std::uint32_t bits = readU32(bytes, at);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void writeU32(std::vector<std::uint8_t>* bytes, std::size_t at,
              std::uint32_t value) {
  std::memcpy(bytes->data() + at, &value, sizeof(value));
}

void writeF32(std::vector<std::uint8_t>* bytes, std::size_t at, float value) {
  std::memcpy(bytes->data() + at, &value, sizeof(value));
}

std::string withChecksum(const std::string& body) {
  std::uint8_t checksum = 0U;
  for (char c : body) checksum ^= static_cast<std::uint8_t>(c);
  const char* hex = "0123456789ABCDEF";
  std::string sentence = "$" + body + "*";
  sentence.push_back(hex[checksum >> 4U]);
  sentence.push_back(hex[checksum & 0x0fU]);
  return sentence + "\r\n";
}

TEST(ControlProtocol, SanitizesAndAlwaysEncodesModeOne) {
  ControlCommand input;
  input.accel = std::numeric_limits<double>::quiet_NaN();
  input.brake = 1.4;
  input.steering = -2.0;
  const SanitizedControl sanitized = ControlCommandSanitizer().sanitize(input);
  EXPECT_TRUE(sanitized.changed);
  EXPECT_DOUBLE_EQ(0.0, sanitized.command.accel);
  EXPECT_DOUBLE_EQ(1.0, sanitized.command.brake);
  EXPECT_DOUBLE_EQ(-1.0, sanitized.command.steering);
  const auto packet = MoraiControlEncoder().encodeMode1(sanitized.command);
  ASSERT_EQ(55U, packet.size());
  EXPECT_EQ(1U, packet[32U]);
  EXPECT_EQ(4U, packet[31U]);
  EXPECT_FLOAT_EQ(0.0F, readF32(packet, 33U));
  EXPECT_FLOAT_EQ(0.0F, readF32(packet, 37U));
}

TEST(ControlProtocol, EncodesMolitComp03BytesExactly) {
  ControlCommand command;
  command.accel = 0.5;
  command.brake = 0.25;
  command.steering = -1.0;
  const std::array<std::uint8_t, 55U> expected{{
      0x23U, 0x4dU, 0x6fU, 0x72U, 0x61U, 0x69U, 0x43U, 0x74U,
      0x72U, 0x6cU, 0x43U, 0x6dU, 0x64U, 0x24U,
      0x17U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x02U, 0x04U, 0x01U,
      0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x3fU,
      0x00U, 0x00U, 0x80U, 0x3eU,
      0x00U, 0x00U, 0x80U, 0xbfU,
      0x0dU, 0x0aU,
  }};
  const auto actual = MoraiControlEncoder().encodeMode1(command);
  ASSERT_EQ(expected.size(), actual.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), actual.begin()));
}

TEST(GpsProtocol, PublishesQualityZeroCoordinatesWithoutInventingCovariance) {
  const std::string sentence = withChecksum("GPGGA,123519,3723.2475,N,12702.3416,E,0,03,5.2,10.0,M,20.0,M,,");
  const GgaFix fix = decodeGgaDatagram(
      reinterpret_cast<const std::uint8_t*>(sentence.data()), sentence.size());
  EXPECT_EQ(0, fix.quality);
  EXPECT_TRUE(fix.has_latitude);
  EXPECT_TRUE(fix.has_longitude);
  EXPECT_NEAR(37.3874583333, fix.latitude, 1e-9);
  EXPECT_NEAR(127.0390266667, fix.longitude, 1e-9);
}

TEST(ImuProtocol, NonFiniteValueDoesNotBecomeStructuralError) {
  std::vector<std::uint8_t> packet(115U, 0U);
  std::memcpy(packet.data(), "#IMUData$", 9U);
  packet[113U] = '\r';
  packet[114U] = '\n';
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::memcpy(packet.data() + 33U, &nan, sizeof(nan));
  const ImuSample sample = decodeImuPacket(packet.data(), packet.size(), PacketLayout::kTimestamped);
  EXPECT_TRUE(std::isnan(sample.orientation_w));
}

TEST(StatusProtocol, DecodesMolitComp03Layout) {
  std::vector<std::uint8_t> packet(181U, 0U);
  std::memcpy(packet.data(), "#MoraiInfo$", 11U);
  writeU32(&packet, 11U, 152U);
  const std::size_t payload = 27U;
  writeU32(&packet, payload + 0U, 123U);
  writeU32(&packet, payload + 4U, 456000000U);
  packet[payload + 8U] = 1U;
  packet[payload + 9U] = 4U;
  writeF32(&packet, payload + 10U, 36.0F);
  writeF32(&packet, payload + 110U, 15.0F);
  std::memcpy(packet.data() + payload + 114U, "LINK_42", 7U);
  packet[179U] = '\r';
  packet[180U] = '\n';

  const VehicleStatusSample sample =
      decodeVehicleStatusPacket(packet.data(), packet.size());
  EXPECT_EQ(123, sample.source_sec);
  EXPECT_EQ(456000000, sample.source_nsec);
  EXPECT_EQ(1U, sample.control_mode);
  EXPECT_EQ(4U, sample.gear);
  EXPECT_FLOAT_EQ(10.0F, sample.signed_velocity_mps);
  EXPECT_NEAR(15.0 * 3.14159265358979323846 / 180.0,
              sample.steering_angle_rad, 1e-6);
  EXPECT_EQ("LINK_42", sample.link_id);
}

TEST(LampProtocol, InvalidFieldsBecomeOff) {
  LampCommand input;
  input.turn_signal = 9;
  input.emergency_signal = -1;
  const SanitizedLamp result = LampCommandSanitizer().sanitize(input);
  EXPECT_TRUE(result.changed);
  EXPECT_EQ(0, result.command.turn_signal);
  EXPECT_EQ(0, result.command.emergency_signal);
}

}  // namespace
}  // namespace morai_udp_bridge
