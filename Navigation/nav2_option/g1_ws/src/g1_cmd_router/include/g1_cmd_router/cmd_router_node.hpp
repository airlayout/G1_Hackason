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
#include <vector>

#include <map>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
    void SendIpcKeepaliveIfIdle();

    // --- STANDBY→READY のゲートと、走行中の鮮度監視(仕様書7章) --------------
    //
    // これまで「SDK に接続できた＝READY」という簡略化をしていた(MVP)。本来は
    // **TF とセンサーが健全であることを確かめてから** READY にしなければならない。
    // 簡略化のままだと、地図が無い/LiDAR が死んでいる状態でも走行を許可してしまう。
    //
    // 監視対象:
    //   - TF: `tf_target_frame` ← `tf_source_frame`(既定 map←base_link)。
    //         **合成された変換を見るので、localization(map→odom)と
    //         state_bridge(odom→base_link)のどちらが落ちても検知できる。**
    //   - センサー: `sensor_topic`(既定 /g1/points_local、PointCloud2)。
    //         local costmap の観測源そのものなので、ここが止まれば
    //         障害物が見えなくなっている。
    void UpdateHealth();
    bool tf_ok() const;
    bool sensor_ok() const;

    void OnSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    // D-31: 操作PC の heartbeat が途絶していないか監視する。途絶したら FAULT。
    void CheckOperatorHeartbeat();

    // Nav2 の実行中 Goal を取り消す。
    //
    // ⚠️ **ゼロ速度の送信とは独立に、best-effort で行う。**
    // キャンセルに失敗しても停止処理は止めない。物理的な停止は
    // ゼロ速度 → SDK側 watchdog → duration 満了 の3層が担保しており、
    // キャンセルは「**復帰後に勝手に巡回が再開しない**」ためのもの。
    //
    // 📌 `nav2_msgs` には依存しない。アクションのキャンセルは
    // `<action>/_action/cancel_goal`(`action_msgs/srv/CancelGoal`) という
    // 汎用サービスで行えるため、`action_msgs`(ros-base に含まれる)だけで済む。
    // これにより `g1_cmd_router` は Nav2 が入っていない環境でもビルド・起動できる。
    void CancelNav2Goals(const std::string& reason);

    void OnEnableNavigation(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void OnStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void OnClearFault(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    // E_STOP の手動解除。⚠️ **自動解除は絶対にしない**(`/g1/estop` に false が
    // 来ても解除しない)。人が明示的にこのサービスを叩くことを要件にする。
    void OnClearEStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
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

    // ⚠️ **SafetyManager(`mgr_`)は必ず実行器スレッドからだけ触る。**
    //
    // 理由が2つある:
    //
    // ① **デッドロック**(2026-09-13 に実測で発覚)。以前は `IpcSend()` が
    //    `ipc_mutex_` を保持したまま `mgr_->OnBridgeDisconnected()` を呼んでおり、
    //    その中の `SendZero()` が `ipc_send_` 経由で `IpcSend()` に**再入**して
    //    同じ非再帰ミューテックスを取りに行っていた。SDK側プロセスが死ぬと
    //    `g1_cmd_router` が丸ごと固まり、**診断の配信が止まり `/g1/estop` も
    //    `/g1/stop` も応答しなくなる**(＝ソフトウェアE-stopが死ぬ)。
    //
    // ② **データ競合**。`mgr_` は素のメンバを持つだけで thread-safe ではない。
    //    再接続スレッドから `OnBridgeConnected()`/`MarkReady()` を呼び、
    //    実行器スレッドから `OnNavTwist()`/`Tick()` を呼んでいた。
    //
    // そこで**他スレッドからは下のフラグを立てるだけ**にし、
    // 状態遷移は `OnTimer()`(50ms)が拾って実行器スレッドで行う。
    // 遅れは最大 50ms で、物理的な停止は SDK側 watchdog と duration 満了が担保する。
    std::atomic<bool> bridge_lost_{false};
    std::atomic<bool> bridge_connected_{false};

    // ⚠️ **IPC の切断は「送信したとき」にしか分からない。**
    // 指令が流れていない間に SDK側プロセスが死んでも、`IpcSend()` が呼ばれないので
    // 気づけず、`READY` のまま「繋がっているつもり」になる(2026-09-13 に実測で発覚)。
    // そこで**送信が途切れたら定期的にゼロ速度を送って生存を確かめる**。
    // ゼロなので機体は動かず、D-10 の「ROS側は明示的ゼロ送信も行う」にも沿う。
    double ipc_keepalive_s_ = 0.2;
    std::optional<rclcpp::Time> last_ipc_send_;

    // --- 健全性監視 ---------------------------------------------------------
    bool require_tf_ = true;
    bool require_sensor_ = true;
    std::string tf_source_frame_;
    std::string tf_target_frame_;
    double tf_timeout_s_ = 0.5;
    std::string sensor_topic_;
    double sensor_timeout_s_ = 1.0;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sensor_;
    std::optional<rclcpp::Time> last_sensor_time_;
    // 直近の判定結果(診断とログの抑制に使う)
    bool tf_ok_ = false;
    bool sensor_ok_ = false;
    std::string tf_reason_;
    bool warned_not_ready_ = false;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_stamped_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_unstamped_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_fault_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_estop_;

    // `/g1/estop` の最後の入力値。**物理の E-stop と同じインターロック**にする:
    // 押しボタンを戻さないとリセットボタンが効かないのと同じで、
    // `/g1/estop` に false を送って「手を離した」状態にしないと解除を受け付けない。
    // これが無いと、true を出し続けている発信源が居るのに解除でき、
    // 次の tick で即座に E_STOP に戻る(あるいは戻らずに走り出す)という
    // 分かりにくい状態になる。
    bool estop_input_ = false;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

    // Nav2 Goal キャンセル用。空なら機能を無効にする。
    std::vector<std::string> nav2_cancel_services_;
    std::vector<rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr> cancel_clients_;

    // --- D1: Goal が走っていない間は cmd_timeout を数えない(2026-09-24) -------
    // ⚠️ **これが無いと、Goal に到達するたびに必ず FAULT に落ちる。** 到達後は
    // Nav2 が指令を出すのをやめ、velocity_smoother も 1 秒で沈黙するため
    // (2026-09-15 / 09-24 の実機でどちらも踏んだ)。巡回は各点で必ずこれを踏む。
    // 📌 **「Nav2 が死んだ」検知は失われない。** Goal が ACTIVE のまま指令が
    // 途切れれば、従来どおり cmd_timeout で FAULT になる。
    // ⚠️ status を一度も受け取れない構成(Nav2 が上がっていない等)では
    // cmd_timeout が働かない。その場合は TF 鮮度とセンサー鮮度が受け持つ。
    // ⚠️ **トピックごとに覚えて OR を取る。** 1つの bool を共有すると、
    // 走っていない側の status(空)が走っている側の true を消してしまう。
    void OnGoalStatus(const std::string& topic,
                      const action_msgs::msg::GoalStatusArray::SharedPtr msg);
    bool AnyGoalActive() const;
    std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> sub_goal_status_;
    std::map<std::string, bool> goal_active_by_topic_;
    bool cmd_timeout_requires_goal_ = true;   // false にすると 2026-09-24 以前の挙動
    bool logged_goal_status_ = false;
    // NAVIGATING から異常系へ抜けたことを検知するために前回の状態を覚えておく
    g1_sdk_bridge::NavState prev_state_ = g1_sdk_bridge::NavState::kDisconnected;
};

}  // namespace g1_cmd_router
