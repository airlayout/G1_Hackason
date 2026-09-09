// SdkBridgeProcess の統合テスト。実スレッド + 実Unix domain socketで検証する。
//
// タイミング系のテストはsleepではなくWaitUntil()でポーリングし、CI環境差による
// flakyさを抑える。ただしスレッド+実ソケットを使うため、他のテストより本質的に
// 時間がかかる(数百ms程度)。

#include "g1_sdk_bridge/sdk_bridge_process.hpp"

#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "g1_sdk_bridge/ipc_transport.hpp"
#include "g1_sdk_bridge/protocol.hpp"

using namespace g1_sdk_bridge;
using namespace std::chrono_literals;

namespace {

void WaitUntil(const std::function<bool()>& predicate, std::chrono::duration<double> timeout = 2.0s,
               std::chrono::duration<double> interval = 10ms, const char* message = "condition not met in time") {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return;
        std::this_thread::sleep_for(interval);
    }
    throw std::runtime_error(message);
}

class TempDir {
public:
    TempDir() {
        char tmpl[] = "/tmp/g1bridge_test_XXXXXX";
        const char* dir = mkdtemp(tmpl);
        if (dir == nullptr) throw std::runtime_error("mkdtemp失敗");
        path_ = dir;
    }
    ~TempDir() {
        std::remove((path_ + "/cmd.sock").c_str());
        std::remove((path_ + "/state.sock").c_str());
        ::rmdir(path_.c_str());
    }
    std::string cmd_path() const { return path_ + "/cmd.sock"; }
    std::string state_path() const { return path_ + "/state.sock"; }

private:
    std::string path_;
};

}  // namespace

// 速いテスト条件(cmd_rate=50Hz)で構成する。実運用値は仕様書の20Hzを想定。
class SdkBridgeTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfg_.cmd_sock_path = tmp_.cmd_path();
        cfg_.state_sock_path = tmp_.state_path();
        cfg_.cmd_rate_hz = 50.0;  // period=0.02s
        cfg_.sdk_command_duration_s = 0.04;
        cfg_.cmd_timeout_s = 0.08;
        cfg_.max_sdk_errors = 3;
        bridge_ = std::make_unique<SdkBridgeProcess>(cfg_, backend_);
        bridge_->Start();
    }

    void TearDown() override { bridge_->Stop(); }

    SeqPacketEndpoint ConnectCmdClient() { return ConnectClient(cfg_.cmd_sock_path, sizeof(CmdWire), 1.0); }
    SeqPacketEndpoint ConnectStateClient() { return ConnectClient(cfg_.state_sock_path, sizeof(StateWire), 1.0); }

    void SendCmd(SeqPacketEndpoint& client, std::uint64_t seq, double vx, double vy, double omega) {
        const CmdPacket pkt{seq, MonotonicNs(), vx, vy, omega};
        const CmdWire wire = pkt.Encode();
        client.SendLatest(&wire, sizeof(wire));
    }

    TempDir tmp_;
    SdkBridgeConfig cfg_;
    MockMoveBackend backend_;
    std::unique_ptr<SdkBridgeProcess> bridge_;
};

TEST_F(SdkBridgeTest, StartupSendsZeroBeforeAnyConnection) {
    // D-11: 起動直後、接続前でも必ずゼロ速度を1回送っている
    const auto first = backend_.AllCalls().front();
    EXPECT_DOUBLE_EQ(first.vx, 0.0);
    EXPECT_DOUBLE_EQ(first.vy, 0.0);
    EXPECT_DOUBLE_EQ(first.omega, 0.0);
    EXPECT_DOUBLE_EQ(first.duration, cfg_.sdk_command_duration_s);
}

TEST_F(SdkBridgeTest, DurationArgumentIsAlwaysFinite) {
    // D-27: 呼び出しの度にdurationを明示していることを確認する(continous_move相当を使わない)
    WaitUntil([this]() { return backend_.CallCount() >= 3; });
    for (const auto& call : backend_.AllCalls()) {
        EXPECT_GT(call.duration, 0.0);
        EXPECT_LE(call.duration, cfg_.cmd_timeout_s);
    }
}

TEST_F(SdkBridgeTest, CmdForwardedToBackend) {
    auto client = ConnectCmdClient();
    WaitUntil([this]() { return bridge_->status() != BridgeStatus::kDisconnected; });
    SendCmd(client, 1, 0.15, 0.0, 0.05);

    WaitUntil([this]() {
        const auto last = backend_.LastCall();
        return last.has_value() && last->vx == 0.15 && last->vy == 0.0 && last->omega == 0.05;
    });
}

TEST_F(SdkBridgeTest, WatchdogZeroesAfterCmdTimeout) {
    auto client = ConnectCmdClient();
    WaitUntil([this]() { return bridge_->status() != BridgeStatus::kDisconnected; });
    SendCmd(client, 1, 0.15, 0.0, 0.0);
    WaitUntil([this]() {
        const auto last = backend_.LastCall();
        return last.has_value() && last->vx == 0.15;
    });

    // それ以降は何も送らない -> cmd_timeout超過でSDK側watchdogがゼロを送るはず
    WaitUntil(
        [this]() {
            const auto last = backend_.LastCall();
            return last.has_value() && last->vx == 0.0 && last->vy == 0.0 && last->omega == 0.0;
        },
        1.0s, 10ms, "cmd_timeout超過後もゼロ速度が送られなかった(SDK側watchdogが機能していない)");
}

TEST_F(SdkBridgeTest, PeerDisconnectSetsStatusDisconnectedAndZero) {
    auto client = ConnectCmdClient();
    WaitUntil([this]() { return bridge_->status() != BridgeStatus::kDisconnected; });
    SendCmd(client, 1, 0.1, 0.0, 0.0);
    WaitUntil([this]() {
        const auto last = backend_.LastCall();
        return last.has_value() && last->vx == 0.1;
    });

    client.Close();
    WaitUntil([this]() { return bridge_->status() == BridgeStatus::kDisconnected; });
    WaitUntil([this]() {
        const auto last = backend_.LastCall();
        return last.has_value() && last->vx == 0.0;
    });
}

TEST_F(SdkBridgeTest, ConsecutiveSdkErrorsTriggerFaultAndStayUntilClear) {
    auto client = ConnectCmdClient();
    WaitUntil([this]() { return bridge_->status() != BridgeStatus::kDisconnected; });

    backend_.FailNext(cfg_.max_sdk_errors);
    SendCmd(client, 1, 0.1, 0.0, 0.0);

    WaitUntil([this]() { return bridge_->status() == BridgeStatus::kFault; }, 2.0s, 10ms,
              "連続SDKエラーでFAULTに遷移しなかった");

    // D-13: FAULT中は新しい有効な指令が来ても自動では復帰しない
    SendCmd(client, 2, 0.1, 0.0, 0.0);
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(bridge_->status(), BridgeStatus::kFault);
    const auto last = backend_.LastCall();
    ASSERT_TRUE(last.has_value());
    EXPECT_DOUBLE_EQ(last->vx, 0.0);

    // 操作者がClearFault()した後は復帰できる
    EXPECT_TRUE(bridge_->ClearFault());
    SendCmd(client, 3, 0.1, 0.0, 0.0);
    WaitUntil([this]() {
        const auto l = backend_.LastCall();
        return l.has_value() && l->vx == 0.1;
    });
}

TEST_F(SdkBridgeTest, SuccessfulCallResetsConsecutiveErrorCount) {
    // 仕様書8章: 「連続」失敗回数。成功が挟まればカウントはリセットされ、FAULTにならない
    auto client = ConnectCmdClient();
    WaitUntil([this]() { return bridge_->status() != BridgeStatus::kDisconnected; });

    for (int i = 0; i < cfg_.max_sdk_errors - 1; ++i) {
        backend_.FailNext(1);
        SendCmd(client, 1, 0.1, 0.0, 0.0);
        WaitUntil([this]() { return bridge_->sdk_error_count() >= 1; });
        WaitUntil([this]() { return bridge_->sdk_error_count() == 0; });  // 次の成功呼び出しでリセットされる
    }

    EXPECT_NE(bridge_->status(), BridgeStatus::kFault);
}

TEST_F(SdkBridgeTest, StateChannelReportsStatusAndPose) {
    auto cmd_client = ConnectCmdClient();
    auto state_client = ConnectStateClient();
    WaitUntil([this]() { return bridge_->status() != BridgeStatus::kDisconnected; });

    SendCmd(cmd_client, 1, 0.1, 0.0, 0.0);

    std::vector<StatePacket> received;
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline && received.size() < 3) {
        auto raw = state_client.RecvLatest();
        if (raw.has_value()) {
            received.push_back(StatePacket::Decode(raw->data(), raw->size()));
        }
        std::this_thread::sleep_for(5ms);
    }

    ASSERT_GE(received.size(), 1u);
    EXPECT_TRUE(std::any_of(received.begin(), received.end(),
                             [](const StatePacket& p) { return p.status == BridgeStatus::kNavigating; }));
    // x=0スタートで前進指令を送っているので、いずれかの時点でxが正に進んでいるはず
    EXPECT_TRUE(std::any_of(received.begin(), received.end(), [](const StatePacket& p) { return p.x > 0.0; }));
}
