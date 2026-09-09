#include "g1_sdk_bridge/safety_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace g1_sdk_bridge {

namespace {
double DefaultNow() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

double ClampScalar(double v, double limit) { return std::max(-limit, std::min(limit, v)); }

double Step(double p, double t, double max_rate, double dt) {
    const double max_delta = max_rate * dt;
    double delta = t - p;
    delta = std::max(-max_delta, std::min(max_delta, delta));
    return p + delta;
}
}  // namespace

Vel Clamp(const Vel& v, const SafetyLimits& limits) {
    return {ClampScalar(v[0], limits.max_vx), ClampScalar(v[1], limits.max_vy), ClampScalar(v[2], limits.max_wz)};
}

Vel ApplyDeadband(const Vel& v, const SafetyLimits& limits) {
    Vel out = v;
    if (std::abs(out[0]) < limits.min_vx) out[0] = 0.0;
    if (std::abs(out[2]) < limits.min_wz) out[2] = 0.0;
    // vy はMVPで無効(D-15)なので同じ扱いにはしない。呼び出し側でmax_vy=0によりclampで既に0になる
    return out;
}

Vel AccelLimit(const Vel& prev, const Vel& target, double dt, const SafetyLimits& limits) {
    if (dt <= 0.0) {
        return prev;
    }
    return {
        Step(prev[0], target[0], limits.max_ax, dt),
        Step(prev[1], target[1], limits.max_ay, dt),
        Step(prev[2], target[2], limits.max_awz, dt),
    };
}

SafetyManager::SafetyManager(SafetyLimits limits, IpcSend ipc_send, NowFn now_fn)
    : limits_(limits), ipc_send_(std::move(ipc_send)), now_(now_fn ? std::move(now_fn) : NowFn(DefaultNow)) {}

void SafetyManager::OnBridgeConnected() {
    if (state_ == NavState::kDisconnected) {
        state_ = NavState::kStandby;
    }
}

void SafetyManager::OnBridgeDisconnected() {
    state_ = NavState::kDisconnected;
    SendZero();
}

bool SafetyManager::EnableNavigation(bool enable) {
    if (enable) {
        if (state_ != NavState::kReady) {
            return false;
        }
        state_ = NavState::kNavigating;
        const double now = now_();
        // last_nav_cmd_time_ はここでは設定しない。Nav2はGoal計画に数百ms〜数秒かかることが
        // あり(実測: 合成マップでのdry-runで約1秒)、enable直後にcmd_timeoutのカウントを
        // 始めると最初の指令が届く前にFAULTへ誤って遷移してしまう(2026-09-09に実機なしの
        // Nav2統合テストで発見)。最初の指令を受け取るまではTick()のタイムアウト判定を
        // 待機させ(下記Tick参照)、一度でも指令を受けたら通常のcmd_timeout監視に切り替える。
        // 物理的な安全性はSDK側watchdog(D-09)が指令を受け取らない限りゼロ速度を送り続ける
        // ことで別途担保されるため、この「最初の指令を待つ」猶予は安全上問題ない。
        last_nav_cmd_time_.reset();  // 前回NAVIGATINGだった時の古い値を引き継がない
        last_tick_time_ = now;  // 有効化した瞬間を基準にAccelLimitのdtを測り始める
        last_output_ = {0.0, 0.0, 0.0};
        return true;
    }
    if (state_ == NavState::kNavigating) {
        state_ = NavState::kReady;
        SendZero();
    }
    return true;
}

void SafetyManager::MarkReady() {
    if (state_ == NavState::kStandby) {
        state_ = NavState::kReady;
    }
}

Vel SafetyManager::OnNavTwist(double vx, double vy, double omega) {
    const double now = now_();
    last_nav_cmd_time_ = now;

    if (state_ != NavState::kNavigating) {
        // 仕様書8章の状態別許可表: NAVIGATING以外では非ゼロ速度を送らない
        return SendZero();
    }

    Vel target = Clamp({vx, vy, omega}, limits_);
    target = ApplyDeadband(target, limits_);
    // last_tick_time_ は EnableNavigation(true) で必ず設定済み(NAVIGATING以外ではここに来ない)。
    // AccelLimit自身がdt<=0を「変化なし」として扱うため、ここで特別扱いはしない。
    const double dt = last_tick_time_.has_value() ? std::max(0.0, now - *last_tick_time_) : 0.0;
    const Vel output = AccelLimit(last_output_, target, dt, limits_);
    last_output_ = output;
    last_tick_time_ = now;
    ipc_send_(output[0], output[1], output[2]);
    return output;
}

void SafetyManager::Tick() {
    if (state_ != NavState::kNavigating) {
        return;
    }
    if (!last_nav_cmd_time_.has_value()) {
        return;  // まだ最初の指令を受けていない。Nav2の計画時間を待つ(上記EnableNavigation参照)
    }
    const double now = now_();
    if (now - *last_nav_cmd_time_ > limits_.cmd_timeout_s) {
        TransitionFault("cmd_timeout");
    }
}

void SafetyManager::EStop() {
    state_ = NavState::kEStop;
    SendZero();
}

bool SafetyManager::ClearEStop() {
    if (state_ != NavState::kEStop) {
        return false;
    }
    state_ = NavState::kStandby;
    fault_reason_.reset();
    return true;
}

void SafetyManager::OnTfStale() { TransitionFault("tf_stale"); }
void SafetyManager::OnSensorStale() { TransitionFault("sensor_stale"); }
void SafetyManager::OnBridgeError() { TransitionFault("sdk_bridge_error"); }
void SafetyManager::OnCollisionStop() { SendZero(); }

bool SafetyManager::ClearFault() {
    if (state_ != NavState::kFault) {
        return false;
    }
    state_ = NavState::kStandby;
    fault_reason_.reset();
    return true;
}

void SafetyManager::TransitionFault(const std::string& reason) {
    if (state_ == NavState::kEStop) {
        return;  // E-stopが最優先。FAULTで上書きしない
    }
    state_ = NavState::kFault;
    fault_reason_ = reason;
    SendZero();
}

Vel SafetyManager::SendZero() {
    last_output_ = {0.0, 0.0, 0.0};
    ipc_send_(0.0, 0.0, 0.0);
    return last_output_;
}

}  // namespace g1_sdk_bridge
