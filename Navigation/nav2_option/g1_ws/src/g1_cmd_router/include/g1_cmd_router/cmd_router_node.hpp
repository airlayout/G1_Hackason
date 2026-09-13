// g1_cmd_router: Nav2からの速度指令を安全処理し、SDK側プロセスへIPC送信するノード。
//
// 仕様書7章(状態機械)・8章(Safety Manager)、Planning.md D-04(2プロセス構成)・
// D-05(IPC)・D-10(watchdog二重化)・D-14(デッドバンド)・D-27(duration明示)に対応。
//
// ロジック本体(g1_sdk_bridge::SafetyManager)はROS 2非依存(g1_sdk_bridge_cpp/)。
// このノードは薄いラッパーとして、トピック購読・サービス提供・IPC送受信の配線だけを行う。

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "g1_sdk_bridge/heartbeat.hpp"
#include "g1_sdk_bridge/ipc_transport.hpp"
#include "g1_sdk_bridge/safety_manager.hpp"

namespace g1_cmd_router {

class CmdRouterNode : public rclcpp::Node {
public:
    CmdRouterNode();
    ~CmdRouterNode() override;

private:
    // Nav2 の `velocity_smoother` が `/cmd_vel_smoothed` に出す型はディストリで異なる:
    //   - Humble: `geometry_msgs/Twist` 固定(`enable_stamped_cmd_vel` 自体が無い)
    //   - Jazzy : 既定 `Twist`。`enable_stamped_cmd_vel=true` で `TwistStamped`(D-23)
    //   - Kilted 以降: 既定 `TwistStamped`
    // 片方だけを購読していると、型が違うディストリと繋いだとき
    // **エラーも警告も出ないまま指令が1件も届かない**(2026-09-13 に実測で確認。
    // `ros2 topic info --verbose` が `Type: ['geometry_msgs/msg/Twist',
    // 'geometry_msgs/msg/TwistStamped']` と2つの型を並べるだけで、購読側は沈黙する)。
    //
    // ⚠️⚠️ **同じトピック名に2つの型を同時購読してはいけない。**
    // 一度その実装にしたが、**`rmw_fastrtps_cpp` では起動時に例外で落ちる**
    // (`create_subscription() called for existing topic name ... with incompatible
    // type`)。`rmw_cyclonedds_cpp` では通ってしまうため amd64 の検証環境では
    // 気づけず、arm64(FastDDS 既定)で初めて発覚した(2026-09-13)。
    // **D-03 は ROS 側 RMW を FastDDS としている**ので、これは致命的だった。
    //
    // そこで **publisher の型を実行時に調べて、合う方を1本だけ張る**。
    // 型が分かるまで(publisher が現れるまで)は購読を作らず、タイマーで待つ。
    void OnTwistStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void OnTwistUnstamped(const geometry_msgs::msg::Twist::SharedPtr msg);
    void OnNavTwist(double vx, double vy, double omega, bool stamped);
    // publisher の型を調べて購読を1本だけ作る。作れたら true。
    bool TryCreateCmdVelSubscription();
    void CreateStampedSubscription();
    void CreateUnstampedSubscription();
    void OnEStop(const std_msgs::msg::Bool::SharedPtr msg);
    void OnTimer();
    void IpcSend(double vx, double vy, double omega);
    void ReconnectLoop();
    void PublishDiagnostics();
    void WarnIfNoCommand();
    // D-31: 操作PC の heartbeat が途絶していないか監視する。途絶したら FAULT。
    void CheckOperatorHeartbeat();

    void OnEnableNavigation(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void OnStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void OnClearFault(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    std::string cmd_sock_path_;
    std::string cmd_vel_topic_;

    // D-31: 操作PC 生存監視。`heartbeat_required=false` のときは nullptr。
    // ⚠️ **既定は有効(true)。** 人と繋がっていない状態で巡回させないため。
    // ベンチでモック相手に動かすときだけ明示的に false にする。
    bool heartbeat_required_ = true;
    std::unique_ptr<g1_sdk_bridge::HeartbeatReceiver> heartbeat_;
    bool warned_heartbeat_missing_ = false;
    g1_sdk_bridge::SafetyLimits limits_;
    std::unique_ptr<g1_sdk_bridge::SafetyManager> mgr_;

    std::mutex ipc_mutex_;
    std::optional<g1_sdk_bridge::SeqPacketEndpoint> cmd_endpoint_;  // ipc_mutex_で保護
    std::uint64_t seq_ = 0;                                        // ipc_mutex_で保護

    // どちらの型で指令が来ているかを一度だけログに出すためのフラグ(配線ミスの早期発見用)
    bool logged_stamped_ = false;
    bool logged_unstamped_ = false;
    // "auto" | "twist" | "twist_stamped"
    std::string cmd_vel_type_;
    // 型が判明するまで publisher を探し続けるタイマー(判明したら解除する)
    rclcpp::TimerBase::SharedPtr resolve_timer_;
    // NAVIGATING に入ってから一度も指令が来ていないことを警告する閾値。
    // **状態遷移はさせない**(FAULT にはしない)。SafetyManager の「最初の指令を待つ」
    // 猶予は意図的な設計なので変えず、**黙って動かない状況を可視化するだけ**にとどめる。
    double no_cmd_warn_s_ = 0.0;
    std::optional<rclcpp::Time> navigating_since_;   // NAVIGATING に入った時刻
    std::optional<rclcpp::Time> last_cmd_time_;      // 最後に指令を受けた時刻
    bool warned_no_cmd_ = false;

    std::atomic<bool> running_{true};
    std::thread reconnect_thread_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_stamped_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_unstamped_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_fault_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
};

}  // namespace g1_cmd_router
