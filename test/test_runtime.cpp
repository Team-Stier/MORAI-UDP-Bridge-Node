#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "morai_udp_bridge/runtime.hpp"

namespace morai_udp_bridge {
namespace {

TEST(ControlUdpSocket, ReceivesReplyOnItsSendSourcePort) {
  ros::Time::init();
  const int peer = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(peer, 0);
  sockaddr_in peer_address{};
  peer_address.sin_family = AF_INET;
  peer_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  peer_address.sin_port = 0;
  ASSERT_EQ(0, ::bind(peer, reinterpret_cast<sockaddr*>(&peer_address),
                      sizeof(peer_address)));
  socklen_t peer_size = sizeof(peer_address);
  ASSERT_EQ(0, getsockname(peer, reinterpret_cast<sockaddr*>(&peer_address),
                           &peer_size));

  ControlUdpSocket socket("127.0.0.1", ntohs(peer_address.sin_port),
                          "127.0.0.1", 0, 65536U, 3U);
  const std::vector<std::uint8_t> request{1U, 2U, 3U};
  int error = 0;
  ASSERT_TRUE(socket.send(request, &error));

  std::array<std::uint8_t, 16U> peer_buffer{};
  sockaddr_in sender{};
  socklen_t sender_size = sizeof(sender);
  ASSERT_EQ(3, recvfrom(peer, peer_buffer.data(), peer_buffer.size(), 0,
                        reinterpret_cast<sockaddr*>(&sender), &sender_size));
  const std::array<std::uint8_t, 4U> reply{{4U, 5U, 6U, 7U}};
  ASSERT_EQ(4, sendto(peer, reply.data(), reply.size(), 0,
                      reinterpret_cast<sockaddr*>(&sender), sender_size));

  Datagram datagram;
  std::array<std::uint8_t, 16U> receive_buffer{};
  int result = -1;
  for (int i = 0; i < 100 && result < 0; ++i) {
    result = socket.receive(receive_buffer.data(), receive_buffer.size(),
                            &datagram, &error);
    if (result < 0 && (error == EAGAIN || error == EWOULDBLOCK))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(1, result);
  EXPECT_EQ(std::vector<std::uint8_t>(reply.begin(), reply.end()),
            datagram.bytes);
  ::close(peer);
}

}  // namespace
}  // namespace morai_udp_bridge
