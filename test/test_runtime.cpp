#include <cerrno>
#include <gtest/gtest.h>
#include "morai_udp_bridge/runtime.hpp"

namespace morai_udp_bridge {

TEST(RuntimeContract, TxRecoveryIsExplicitAndChannelLocal) {
    TxSocketRecoveryPolicy policy(3);
    EXPECT_EQ(SocketAction::kKeep, policy.classify(ENETUNREACH, 1));
    EXPECT_EQ(SocketAction::kReopen, policy.classify(ENETUNREACH, 3));
    EXPECT_EQ(SocketAction::kReopen, policy.classify(EBADF, 1));
    UdpSender failed("127.0.0.1", 9000);
    UdpSender independent("127.0.0.1", 9001);
    int error = 0;
    EXPECT_FALSE(failed.send(std::vector<std::uint8_t>(65536U), &error));
    EXPECT_EQ(1U, failed.consecutiveErrors());
    EXPECT_EQ(EMSGSIZE, error);
    EXPECT_EQ(0U, independent.consecutiveErrors());
    EXPECT_TRUE(failed.reopen());
    EXPECT_EQ(0U, failed.consecutiveErrors());
}

TEST(RuntimeContract, SelectorUsesNewestDecodableThenBatchIsDiscarded) {
    std::vector<Datagram> batch(3);
    batch[0].bytes = {1};
    batch[1].bytes = {2};
    batch[2].bytes = {};
    int sample = 0;
    std::size_t selected = 0, rejected = 0, non_target = 0;
    auto parse = [](const Datagram& datagram) {
        if (datagram.bytes.empty()) throw ProtocolError("short packet");
        return datagram.bytes[0];
    };
    ASSERT_TRUE(LatestDatagramSelector::selectNewestDecodable(
        batch, parse, &sample, &selected, &rejected, &non_target));
    EXPECT_EQ(2, sample);
    EXPECT_EQ(1U, selected);
    EXPECT_EQ(1U, rejected);
    batch.clear();
    EXPECT_FALSE(LatestDatagramSelector::selectNewestDecodable(
        batch, parse, &sample, &selected, &rejected, &non_target));
}

}  // namespace morai_udp_bridge
