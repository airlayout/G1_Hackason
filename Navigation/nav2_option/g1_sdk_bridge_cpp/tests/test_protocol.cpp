#include "g1_sdk_bridge/protocol.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

using namespace g1_sdk_bridge;

TEST(CmdPacket, Roundtrip) {
    const CmdPacket pkt = MakeCmd(42, 0.2, 0.0, -0.1);
    const CmdWire wire = pkt.Encode();
    const CmdPacket decoded = CmdPacket::Decode(&wire, sizeof(wire));
    EXPECT_TRUE(decoded == pkt);
}

TEST(CmdPacket, DecodeRejectsWrongSize) {
    std::vector<std::uint8_t> raw(4, 0);
    EXPECT_THROW(CmdPacket::Decode(raw.data(), raw.size()), ProtocolError);
}

TEST(CmdPacket, DecodeRejectsBadMagic) {
    const CmdPacket pkt = MakeCmd(1, 0.0, 0.0, 0.0);
    CmdWire wire = pkt.Encode();
    wire.magic ^= 0xFFFFFFFFu;
    EXPECT_THROW(CmdPacket::Decode(&wire, sizeof(wire)), ProtocolError);
}

TEST(CmdPacket, AgeSecondsIsNonNegativeAndGrows) {
    const CmdPacket pkt = MakeCmd(1, 0.0, 0.0, 0.0);
    const std::uint64_t now = pkt.timestamp_ns;
    EXPECT_DOUBLE_EQ(pkt.AgeSeconds(now), 0.0);
    EXPECT_NEAR(pkt.AgeSeconds(now + 100'000'000ULL), 0.1, 1e-6);
}

TEST(CmdPacket, AgeSecondsClampedToZeroIfFuture) {
    // 時刻の巻き戻り(単調時計なので理論上起きないはずだが)に対しても負にならないことを保証する
    const CmdPacket pkt = MakeCmd(1, 0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(pkt.AgeSeconds(pkt.timestamp_ns - 1'000'000ULL), 0.0);
}

TEST(StatePacket, Roundtrip) {
    StatePacket pkt;
    pkt.seq = 7;
    pkt.timestamp_ns = 123456789;
    pkt.status = BridgeStatus::kNavigating;
    pkt.x = 1.5;
    pkt.y = -2.5;
    pkt.yaw = 0.78;
    pkt.vx = 0.1;
    pkt.vy = 0.0;
    pkt.omega = 0.05;
    pkt.sdk_error_count = 2;

    const StateWire wire = pkt.Encode();
    const StatePacket decoded = StatePacket::Decode(&wire, sizeof(wire));
    EXPECT_TRUE(decoded == pkt);
}

TEST(StatePacket, DecodeRejectsWrongSize) {
    std::vector<std::uint8_t> raw(10, 0);
    EXPECT_THROW(StatePacket::Decode(raw.data(), raw.size()), ProtocolError);
}
