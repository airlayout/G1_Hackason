// ROS側 Safety Manager / Command Router のロジック本体(仕様書7章・8章に対応)。
//
// Python版プロトタイプ(../../g1_sdk_bridge/safety_manager.py)の1対1移植。
// rclcppに依存しない純粋ロジックとして実装する。実際のrclcppノードは、これを
// ラップしてトピック/サービスに接続するだけになる想定。
//
// Planning.md の対応する決定事項:
// - D-10: watchdogは二重化する。ROS側は「明示的ゼロ送信」+「送信停止」の両方を行う
// - D-13: 異常時はゼロ速度を優先し、Damp/ZeroTorqueへ自動遷移させない
// - D-14: 速度デッドバンド(min_vx / min_wz)を追加する
// - D-15: MVPはvy=0(横移動無効)から開始する

#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>

namespace g1_sdk_bridge {

// 仕様書7章の状態機械。
enum class NavState {
    kDisconnected,
    kStandby,
    kReady,
    kNavigating,
    kFault,
    kEStop,
};

// 仕様書8章「初期安全パラメータ」に対応。
//
// ⚠️ **速度に関する値は 2026-09-09 の実機実測に基づく。加速度はまだ仮値。**
//
// | 指令 vx | 実機の挙動（実測）|
// |---|---|
// | 0.1 m/s | **歩容が成立しない**（関節が 0.4° 動く姿勢微応答のみ）|
// | 0.2 m/s | **歩くが進行方向が定まらない**（実際の進行方向が機体前方から +98.6° ずれた）|
// | 0.3 m/s | **実用的に前進する**（3回とも Δx +0.228〜0.231m、方向のずれ -17.7〜-20.4°）|
//
// したがって**使える前進速度の帯は 0.25〜0.30 m/s と狭い**。0.30 超は未測定。
// この帯の外では「動かない」か「意図しない方向へ進む」ので、
// max_vx と min_vx の両方で囲い込む必要がある。
struct SafetyLimits {
    // 0.30: 実測で確実に前進する速度。**これより上は未測定なので上げないこと**
    double max_vx = 0.30;
    double max_vy = 0.0;  // D-15: MVPでは横移動無効
    // 0.30: omega=+0.3 で +10.20°/1秒を実測。これより上は未測定
    double max_wz = 0.30;
    // ⚠️ 加速度は**未測定の仮値**。実測すべき(Phase 1 項目9 の残作業)
    double max_ax = 0.20;
    double max_ay = 0.15;
    double max_awz = 0.40;
    // D-14: これ未満はデッドバンドでゼロに丸める(歩容が成立しない速度域)。
    //
    // **2026-09-09 の U-12 実測により min_vx を有効化した(0.0 → 0.25)。**
    // 0.2 m/s では歩容は成立するのに**進行方向が機体前方から 98.6° ずれた**ため、
    // D-14 のデッドバンドは「足踏みを防ぐ」だけでなく
    // **「進行方向が定まらない速度域を使わせない」ためにも必要**だと分かった。
    // 閾値の正確な境界は 0.2〜0.3 の間で未確定なので、安全側の 0.25 を採る。
    //
    // ⚠️ min_wz は **0.0 のまま**。旋回の下限は未測定。
    // かつて疑似データの dry-run で Nav2 の rotate-to-heading(0.02 rad/s)を
    // デッドバンドが潰して動き出せなくなる不具合を出したが、**実機構成では
    // rotate-to-heading が 0.300 rad/s 出ている**(A-10 実測)ので、
    // 旋回の下限を測ってから設定すればその問題は起きにくい。
    double min_vx = 0.25;
    double min_wz = 0.0;
    double cmd_timeout_s = 0.30;
    int max_sdk_errors = 3;
};

using Vel = std::array<double, 3>;  // {vx, vy, omega}

Vel Clamp(const Vel& v, const SafetyLimits& limits);

// D-14: 微小速度をゼロに丸める。
//
// Goal到達直前にNav2が出す微小速度を素通しすると、二足は歩容が成立せず
// 足踏みし続けて到達判定(速度閾値以下を1秒継続)に入れなくなる。
// 加えて 2026-09-09 の実測で、0.2 m/s では**歩容は成立するのに進行方向が
// 98.6° ずれる**ことが分かった。デッドバンドは「方向が定まらない速度域を
// 使わせない」ためにも要る。
//
// ⚠️ **適用順序が重要**: `OnNavTwist` は Clamp → **ApplyDeadband** → AccelLimit の順で
// 処理する。デッドバンドは**目標値**に適用され、AccelLimit がランプアップする
// 中間値には再適用されない。この順序でなければ、min_vx を有効にした瞬間に
// 「ランプアップ中の小さな値が毎回ゼロにされて永久に動き出せない」
// にらみ合いが起きる(A-9 で疑似データを相手に実際に踏んだ不具合)。
Vel ApplyDeadband(const Vel& v, const SafetyLimits& limits);

// 1制御周期あたりの変化量を加速度上限で制限する(仕様書8章 max_ax/max_ay/max_awz)。
Vel AccelLimit(const Vel& prev, const Vel& target, double dt, const SafetyLimits& limits);

using IpcSend = std::function<void(double vx, double vy, double omega)>;
using NowFn = std::function<double()>;  // 秒単位の単調増加時刻

// 優先順位(仕様書8章)を実装する。
//
// E-stop > 通信/SDK/TF/センサー異常 > Nav2指令 の順で、下位の入力を上書きする。
// NAVIGATING状態のときだけ非ゼロ速度がIPCへ送られる(仕様書8章の状態別許可表)。
class SafetyManager {
public:
    SafetyManager(SafetyLimits limits, IpcSend ipc_send, NowFn now_fn = nullptr);

    // --- Bridge接続状態 ------------------------------------------------
    void OnBridgeConnected();
    void OnBridgeDisconnected();

    // --- Navigation Enable/Disable (仕様書5.2 /g1/enable_navigation) ----
    bool EnableNavigation(bool enable);

    // 歩行可能・センサー正常が確認できたときにSTANDBY->READYへ(仕様書7章)。
    void MarkReady();

    // --- Nav2からの速度指令(仕様書5.1 /cmd_vel_smoothed相当の入口) -------
    // Nav2からのTwistを受け、安全処理後にIPCへ送る。戻り値は実際に送った値(テスト用)。
    Vel OnNavTwist(double vx, double vy, double omega);

    // --- 定期watchdog(D-10: ROS側の明示的ゼロ送信 + 送信停止) -----------
    // タイマーで周期呼び出しする(ROS 2ではWallTimer相当)。
    void Tick();

    // --- 異常系(仕様書8章の停止条件) ------------------------------------
    void EStop();
    // 手動解除+安全確認(仕様書7章 E_STOP -> STANDBY)。呼び出し側が安全確認済みであること。
    bool ClearEStop();
    void OnTfStale();
    void OnSensorStale();
    void OnBridgeError();
    // Collision Monitorからの停止指令。FAULTにはせず、その場でゼロにするだけ(再開可能)。
    void OnCollisionStop();
    // 仕様書5.2 /g1/clear_fault。原因解消・安全確認後に呼ぶ。
    bool ClearFault();

    NavState state() const { return state_; }
    std::optional<std::string> fault_reason() const { return fault_reason_; }
    const SafetyLimits& limits() const { return limits_; }
    void set_limits(const SafetyLimits& limits) { limits_ = limits; }

private:
    void TransitionFault(const std::string& reason);
    Vel SendZero();

    SafetyLimits limits_;
    IpcSend ipc_send_;
    NowFn now_;
    NavState state_ = NavState::kDisconnected;
    std::optional<double> last_nav_cmd_time_;
    Vel last_output_{0.0, 0.0, 0.0};
    std::optional<double> last_tick_time_;
    std::optional<std::string> fault_reason_;
};

}  // namespace g1_sdk_bridge
