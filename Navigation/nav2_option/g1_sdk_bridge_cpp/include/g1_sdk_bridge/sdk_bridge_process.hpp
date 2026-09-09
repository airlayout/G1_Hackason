// SDK側プロセス(systemd常駐想定)のロジック本体。
//
// Python版プロトタイプ(../../g1_sdk_bridge/sdk_process_mock.py)の1対1移植。
// 実際のUnitree呼び出しは MoveBackend として抽象化してあり、実機投入時は
// unitree_sdk2(C++)の LocoClient::SetVelocity() を叩く実装に差し替えるだけでよい設計。
//
// Planning.md の対応する決定事項:
// - D-06: SDK側プロセスはROS 2を一切初期化・リンクしない(このファイルはrclcppに依存しない)
// - D-09: 20Hz周期送信とwatchdogはSDK側プロセスに置く
// - D-11: 起動直後に必ずゼロ速度を送信する
// - D-13: 異常時はゼロ速度を優先し、自動でDamp/ZeroTorqueへ遷移させない
// - D-27: SetVelocity()を直接呼び、durationを明示指定する。continous_moveは使わない

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "g1_sdk_bridge/ipc_transport.hpp"
#include "g1_sdk_bridge/protocol.hpp"

namespace g1_sdk_bridge {

// G1への実移動指令の抽象。実機では unitree_sdk2 の LocoClient::SetVelocity() を叩く。
class MoveBackend {
public:
    virtual ~MoveBackend() = default;
    // 成功時は何も返さない。SDK呼び出し失敗時は例外を送出する。
    virtual void SetVelocity(double vx, double vy, double omega, double duration) = 0;
};

// テスト用。呼び出し履歴を記録し、意図的に失敗させることもできる。
//
// SetVelocity()はSdkBridgeProcessの周期スレッドから、LastCall()/CallCount()はテストスレッドから
// 呼ばれるため、Python版(GILに暗黙に守られていた)とは異なりC++では明示的にmutexで保護する。
class MockMoveBackend : public MoveBackend {
public:
    struct Call {
        double vx;
        double vy;
        double omega;
        double duration;
        std::uint64_t timestamp_ns;
    };

    void SetVelocity(double vx, double vy, double omega, double duration) override;
    void FailNext(int n);
    std::optional<Call> LastCall() const;
    std::vector<Call> AllCalls() const;
    std::size_t CallCount() const;

private:
    mutable std::mutex mutex_;
    std::vector<Call> calls_;
    int fail_remaining_ = 0;
};

struct SdkBridgeConfig {
    std::string cmd_sock_path;
    std::string state_sock_path;
    double cmd_rate_hz = 20.0;
    double cmd_timeout_s = 0.30;         // 仕様書8章 cmd_timeout。この値以下にduration(下記)を収める
    double sdk_command_duration_s = 0.20;  // D-27。送信周期(1/cmd_rate_hz)より長く、cmd_timeout以下
    int max_sdk_errors = 3;
    double poll_interval_s = 0.005;

    // sdk_command_duration_s は 送信周期(1/cmd_rate_hz) より長く、cmd_timeout_s 以下でなければ
    // ならない(D-27の原則)。違反時はstd::invalid_argumentを送出する。
    void Validate() const;
};

// systemd常駐を想定したSDK側プロセスのロジック本体。
class SdkBridgeProcess {
public:
    SdkBridgeProcess(SdkBridgeConfig config, MoveBackend& move_backend);
    ~SdkBridgeProcess();

    SdkBridgeProcess(const SdkBridgeProcess&) = delete;
    SdkBridgeProcess& operator=(const SdkBridgeProcess&) = delete;

    void Start();
    void Stop();

    // /g1/clear_fault 相当。原因解消・安全確認後に操作者が呼ぶ(D-13: 自動復帰はしない)。
    bool ClearFault();

    BridgeStatus status() const;
    int sdk_error_count() const;

private:
    void AcceptLoop();
    void CmdRecvLoop();
    void PeriodicLoop();
    void Tick(double dt);
    int ApplyMove(const std::array<double, 3>& effective);
    void PublishState(BridgeStatus status, const std::array<double, 3>& pose, const std::array<double, 3>& effective,
                       int err);

    SdkBridgeConfig cfg_;
    MoveBackend& move_;

    SeqPacketServer cmd_server_;
    SeqPacketServer state_server_;
    std::optional<SeqPacketEndpoint> cmd_endpoint_;
    std::optional<SeqPacketEndpoint> state_endpoint_;

    mutable std::mutex mutex_;
    std::optional<CmdPacket> last_cmd_;
    BridgeStatus status_ = BridgeStatus::kDisconnected;
    int sdk_error_count_ = 0;
    std::uint64_t state_seq_ = 0;
    std::array<double, 3> pose_{0.0, 0.0, 0.0};  // x, y, yaw (デモ用の簡易積分。実機ではLIOが供給する)

    std::atomic<bool> running_{false};
    std::thread accept_thread_;
    std::thread cmd_recv_thread_;
    std::thread periodic_thread_;
};

}  // namespace g1_sdk_bridge
