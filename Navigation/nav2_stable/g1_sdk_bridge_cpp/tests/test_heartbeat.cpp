// 操作PC 生存監視(heartbeat)の単体テスト。Planning.md D-31。

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "g1_sdk_bridge/heartbeat.hpp"
#include "g1_sdk_bridge/safety_manager.hpp"

using namespace g1_sdk_bridge;

namespace {

constexpr std::uint64_t kSession = 0xABCDEF0123456789ull;

std::array<std::uint8_t, kHeartbeatWireSize> MakeWire(std::uint64_t seq, std::uint64_t ns = 12345,
                                                     std::uint64_t session = kSession) {
    HeartbeatPacket pkt;
    pkt.session_id = session;
    pkt.seq = seq;
    pkt.send_monotonic_ns = ns;
    std::array<std::uint8_t, kHeartbeatWireSize> buf{};
    pkt.Encode(buf.data());
    return buf;
}

}  // namespace

// --- ワイヤフォーマット -----------------------------------------------------

TEST(HeartbeatPacketTest, RoundTrip) {
    const auto wire = MakeWire(42, 987654321);
    const auto got = HeartbeatPacket::Decode(wire.data(), wire.size());
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->session_id, kSession);
    EXPECT_EQ(got->seq, 42u);
    EXPECT_EQ(got->send_monotonic_ns, 987654321u);
}

TEST(HeartbeatPacketTest, EncodingIsExplicitLittleEndian) {
    // 別マシン間を渡るので、バイト配置を固定していることを明示的に確かめる。
    const auto wire = MakeWire(1, 0);
    EXPECT_EQ(wire[0], 0x42);  // kHeartbeatMagic = 0x47314842 のリトルエンディアン
    EXPECT_EQ(wire[1], 0x48);
    EXPECT_EQ(wire[2], 0x31);
    EXPECT_EQ(wire[3], 0x47);
    EXPECT_EQ(wire[4], kHeartbeatVersion);
    EXPECT_EQ(wire[5], 0x89);   // session_id の最下位バイト
    EXPECT_EQ(wire[13], 1);     // seq の最下位バイト
}

TEST(HeartbeatPacketTest, RejectsWrongSizeMagicAndVersion) {
    auto wire = MakeWire(1);
    EXPECT_FALSE(HeartbeatPacket::Decode(wire.data(), wire.size() - 1).has_value());
    EXPECT_FALSE(HeartbeatPacket::Decode(wire.data(), wire.size() + 1).has_value());

    auto bad_magic = MakeWire(1);
    bad_magic[0] ^= 0xFF;
    EXPECT_FALSE(HeartbeatPacket::Decode(bad_magic.data(), bad_magic.size()).has_value());

    auto bad_version = MakeWire(1);
    bad_version[4] = 99;
    EXPECT_FALSE(HeartbeatPacket::Decode(bad_version.data(), bad_version.size()).has_value());
}

// --- 監視ロジック -----------------------------------------------------------

TEST(HeartbeatMonitorTest, NotAliveBeforeFirstPacket) {
    double t = 100.0;
    HeartbeatMonitor mon(1.0, [&t]() { return t; });
    // ⚠️ 一度も受けていない状態を「生きている」と扱わないこと
    EXPECT_FALSE(mon.EverReceived());
    EXPECT_FALSE(mon.Alive());
    EXPECT_FALSE(mon.SecondsSinceLast().has_value());
}

TEST(HeartbeatMonitorTest, AliveWhileWithinTimeoutThenDies) {
    double t = 100.0;
    HeartbeatMonitor mon(1.0, [&t]() { return t; });
    const auto wire = MakeWire(1);
    ASSERT_TRUE(mon.OnDatagram(wire.data(), wire.size()));
    EXPECT_TRUE(mon.Alive());

    t = 100.9;
    EXPECT_TRUE(mon.Alive());
    t = 101.0;
    EXPECT_TRUE(mon.Alive()) << "ちょうど timeout の瞬間はまだ生存扱い";
    t = 101.01;
    EXPECT_FALSE(mon.Alive());
    EXPECT_NEAR(*mon.SecondsSinceLast(), 1.01, 1e-9);
}

TEST(HeartbeatMonitorTest, RecoversWhenPacketsResume) {
    double t = 100.0;
    HeartbeatMonitor mon(1.0, [&t]() { return t; });
    auto w1 = MakeWire(1);
    mon.OnDatagram(w1.data(), w1.size());
    t = 105.0;
    ASSERT_FALSE(mon.Alive());
    // リンクが復旧して届き始めたら、監視としては生存に戻る。
    // ⚠️ ただし SafetyManager 側は FAULT のままで、復帰には clear_fault が要る。
    auto w2 = MakeWire(2);
    ASSERT_TRUE(mon.OnDatagram(w2.data(), w2.size()));
    EXPECT_TRUE(mon.Alive());
}

TEST(HeartbeatMonitorTest, RejectsStaleAndDuplicateSeq) {
    double t = 100.0;
    HeartbeatMonitor mon(1.0, [&t]() { return t; });
    auto w5 = MakeWire(5);
    ASSERT_TRUE(mon.OnDatagram(w5.data(), w5.size()));

    t = 100.5;
    // UDP は順序を保証しないので、遅れて届いた古いパケットを受理してはいけない。
    // これを受理すると「実は途絶しているのに生きている」と誤認する。
    auto w3 = MakeWire(3);
    EXPECT_FALSE(mon.OnDatagram(w3.data(), w3.size()));
    auto dup = MakeWire(5);
    EXPECT_FALSE(mon.OnDatagram(dup.data(), dup.size()));
    EXPECT_EQ(mon.rejected(), 2u);

    // 古いパケットで last_rx_time_ が更新されていないこと
    t = 101.6;
    EXPECT_FALSE(mon.Alive());
}

TEST(HeartbeatMonitorTest, AcceptsRestartedSenderWithLowerSeq) {
    // ⚠️ **実装中に踏んだバグ。** 送信プログラムを再起動すると seq が 1 に戻る。
    // session_id で区別しないと受信側が「古いパケット」として永久に拒否し続け、
    // **運用で送信を再起動したらロボットが二度と動けなくなる。**
    double t = 100.0;
    HeartbeatMonitor mon(1.0, [&t]() { return t; });
    auto w100 = MakeWire(100, 0, 0x1111111111111111ull);
    ASSERT_TRUE(mon.OnDatagram(w100.data(), w100.size()));

    t = 105.0;
    ASSERT_FALSE(mon.Alive());

    // 送信側を再起動した → session_id が変わり seq は 1 から
    auto w1 = MakeWire(1, 0, 0x2222222222222222ull);
    EXPECT_TRUE(mon.OnDatagram(w1.data(), w1.size()))
        << "session_id が変わったら seq が小さくても受理すること";
    EXPECT_TRUE(mon.Alive());
    EXPECT_EQ(mon.session_id(), 0x2222222222222222ull);

    // 新しいセッション内では引き続き単調性を要求する
    auto stale = MakeWire(1, 0, 0x2222222222222222ull);
    EXPECT_FALSE(mon.OnDatagram(stale.data(), stale.size()));
}

TEST(HeartbeatMonitorTest, CountsGarbageAsRejected) {
    HeartbeatMonitor mon(1.0);
    const char junk[] = "hello";
    EXPECT_FALSE(mon.OnDatagram(junk, sizeof(junk)));
    EXPECT_EQ(mon.rejected(), 1u);
    EXPECT_EQ(mon.accepted(), 0u);
    EXPECT_FALSE(mon.Alive());
}

TEST(HeartbeatMonitorTest, RejectsNonPositiveTimeout) {
    EXPECT_THROW(HeartbeatMonitor(0.0), std::invalid_argument);
    EXPECT_THROW(HeartbeatMonitor(-1.0), std::invalid_argument);
}

// --- UDP の実際の疎通 -------------------------------------------------------

TEST(HeartbeatReceiverTest, ReceivesRealUdpDatagrams) {
    // ポート 0 は使えない(bind 後に実ポートを取り直す口を用意していない)ため、
    // 衝突しにくい番号を直接指定する。
    constexpr std::uint16_t kPort = 47399;
    HeartbeatReceiver rx(kPort, 1.0, "127.0.0.1");
    rx.Start();
    EXPECT_FALSE(rx.EverReceived());
    EXPECT_FALSE(rx.Alive());

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(fd, 0);
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(kPort);
    ASSERT_EQ(::inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr), 1);

    const auto wire = MakeWire(1);
    ASSERT_EQ(::sendto(fd, wire.data(), wire.size(), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)),
              static_cast<ssize_t>(wire.size()));

    // 受信スレッドが処理するのを待つ
    for (int i = 0; i < 100 && !rx.EverReceived(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(rx.EverReceived());
    EXPECT_TRUE(rx.Alive());
    EXPECT_EQ(rx.accepted(), 1u);

    ::close(fd);
    rx.Stop();
}

TEST(HeartbeatReceiverTest, ThrowsWhenPortIsAlreadyBound) {
    constexpr std::uint16_t kPort = 47398;
    HeartbeatReceiver first(kPort, 1.0, "127.0.0.1");
    // 同じポートを二重に開けたら、黙って動かないのではなく例外で気づけること
    EXPECT_THROW(HeartbeatReceiver(kPort, 1.0, "127.0.0.1"), std::runtime_error);
}

// --- SafetyManager との接続 -------------------------------------------------

TEST(SafetyManagerOperatorLostTest, TransitionsToFaultAndSendsZero) {
    std::vector<std::array<double, 3>> sent;
    SafetyLimits limits;
    double t = 0.0;
    SafetyManager mgr(limits, [&sent](double vx, double vy, double wz) { sent.push_back({vx, vy, wz}); },
                      [&t]() { return t; });
    mgr.OnBridgeConnected();
    mgr.MarkReady();
    ASSERT_TRUE(mgr.EnableNavigation(true));
    mgr.OnNavTwist(0.30, 0.0, 0.0);
    ASSERT_EQ(mgr.state(), NavState::kNavigating);
    sent.clear();

    mgr.OnOperatorLost();

    EXPECT_EQ(mgr.state(), NavState::kFault);
    ASSERT_EQ(mgr.fault_reason().value_or(""), "operator_lost");
    ASSERT_FALSE(sent.empty());
    EXPECT_EQ(sent.back(), (std::array<double, 3>{0.0, 0.0, 0.0}));
}

TEST(SafetyManagerOperatorLostTest, DoesNotOverrideEStop) {
    SafetyLimits limits;
    SafetyManager mgr(limits, [](double, double, double) {});
    mgr.OnBridgeConnected();
    mgr.MarkReady();
    mgr.EStop();
    mgr.OnOperatorLost();
    // E-stop が最優先。通信断で FAULT に書き換えてしまうと、
    // clear_fault だけで復帰できてしまい E-stop の手動解除要件が崩れる。
    EXPECT_EQ(mgr.state(), NavState::kEStop);
}

TEST(SafetyManagerOperatorLostTest, RequiresClearFaultToRecover) {
    SafetyLimits limits;
    SafetyManager mgr(limits, [](double, double, double) {});
    mgr.OnBridgeConnected();
    mgr.MarkReady();
    ASSERT_TRUE(mgr.EnableNavigation(true));
    mgr.OnOperatorLost();
    ASSERT_EQ(mgr.state(), NavState::kFault);

    // 「操作PCが戻ってきた」だけでは再開しない。無人での自動再開を防ぐため。
    EXPECT_FALSE(mgr.EnableNavigation(true));
    EXPECT_EQ(mgr.state(), NavState::kFault);

    ASSERT_TRUE(mgr.ClearFault());
    EXPECT_EQ(mgr.state(), NavState::kStandby);
}
