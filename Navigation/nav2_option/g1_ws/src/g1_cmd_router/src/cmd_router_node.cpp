#include "g1_cmd_router/cmd_router_node.hpp"

#include <chrono>

#include "g1_sdk_bridge/protocol.hpp"

using namespace std::chrono_literals;
using g1_sdk_bridge::CmdPacket;
using g1_sdk_bridge::CmdWire;
using g1_sdk_bridge::ConnectClient;
using g1_sdk_bridge::MonotonicNs;
using g1_sdk_bridge::NavState;
using g1_sdk_bridge::PeerClosed;
using g1_sdk_bridge::SafetyLimits;
using g1_sdk_bridge::SafetyManager;
using g1_sdk_bridge::SeqPacketEndpoint;

namespace g1_cmd_router {

namespace {
const char* NavStateName(NavState s) {
    switch (s) {
        case NavState::kDisconnected: return "DISCONNECTED";
        case NavState::kStandby: return "STANDBY";
        case NavState::kReady: return "READY";
        case NavState::kNavigating: return "NAVIGATING";
        case NavState::kFault: return "FAULT";
        case NavState::kEStop: return "E_STOP";
    }
    return "UNKNOWN";
}
}  // namespace

CmdRouterNode::CmdRouterNode() : rclcpp::Node("g1_cmd_router") {
    // 仕様書8章「初期安全パラメータ」に対応するデフォルト値。実測前の仮値(Planning.md D-16)。
    cmd_sock_path_ = declare_parameter<std::string>("cmd_sock_path", "/tmp/g1_bridge/cmd.sock");
    limits_.max_vx = declare_parameter<double>("max_vx", 0.30);   // 2026-09-09 実測
    limits_.max_vy = declare_parameter<double>("max_vy", 0.0);  // D-15: MVPは横移動無効
    limits_.max_wz = declare_parameter<double>("max_wz", 0.30);
    limits_.max_ax = declare_parameter<double>("max_ax", 0.20);
    limits_.max_ay = declare_parameter<double>("max_ay", 0.15);
    limits_.max_awz = declare_parameter<double>("max_awz", 0.40);
    // D-14: 既定値は0(無効)。Phase 1のU-12実測でデッドバンド閾値が判明するまでの暫定措置
    // (QUESTIONS.md Q8で選択肢を整理し、ユーザーが(d)を選択。2026-09-09)。
    limits_.min_vx = declare_parameter<double>("min_vx", 0.25);   // 2026-09-09 実測(U-12)
    limits_.min_wz = declare_parameter<double>("min_wz", 0.0);
    limits_.cmd_timeout_s = declare_parameter<double>("cmd_timeout", 0.30);
    limits_.max_sdk_errors = declare_parameter<int>("max_sdk_errors", 3);

    mgr_ = std::make_unique<SafetyManager>(limits_, [this](double vx, double vy, double omega) {
        IpcSend(vx, vy, omega);
    });

    // D-23 では TwistStamped に統一する方針だが、**Nav2 側が出す型はディストリで異なる**
    // (Humble は Twist 固定)。両方購読して、どちらで来ても動くようにする。詳細はヘッダのコメント。
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_smoothed");
    no_cmd_warn_s_ = declare_parameter<double>("no_cmd_warn_s", 3.0);
    sub_twist_stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, 10,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) { OnTwistStamped(msg); });
    sub_twist_unstamped_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { OnTwistUnstamped(msg); });

    // 仕様書5.1 /g1/estop。trueでソフトウェア緊急停止する。falseでの自動解除は行わない
    // (E_STOPからの復帰は手動解除+安全確認が必須、仕様書7章)。復帰用サービスは未実装(既知の残作業)。
    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
        "/g1/estop", 10, [this](std_msgs::msg::Bool::SharedPtr msg) { OnEStop(msg); });

    // D-10: ROS側watchdogのtick。cmd_rateと同程度の周期で回す(仕様書11章の例では20Hz)。
    timer_ = create_wall_timer(50ms, [this]() { OnTimer(); });

    srv_enable_ = create_service<std_srvs::srv::SetBool>(
        "/g1/enable_navigation",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) { OnEnableNavigation(req, res); });
    srv_stop_ = create_service<std_srvs::srv::Trigger>(
        "/g1/stop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) { OnStop(req, res); });
    srv_clear_fault_ = create_service<std_srvs::srv::Trigger>(
        "/g1/clear_fault", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) { OnClearFault(req, res); });

    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/g1/bridge_status", 10);

    reconnect_thread_ = std::thread(&CmdRouterNode::ReconnectLoop, this);

    RCLCPP_INFO(get_logger(), "g1_cmd_router 起動。cmd_sock_path=%s", cmd_sock_path_.c_str());
}

CmdRouterNode::~CmdRouterNode() {
    running_ = false;
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
}

void CmdRouterNode::OnTwistStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    OnNavTwist(msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z, true);
}

void CmdRouterNode::OnTwistUnstamped(const geometry_msgs::msg::Twist::SharedPtr msg) {
    OnNavTwist(msg->linear.x, msg->linear.y, msg->angular.z, false);
}

void CmdRouterNode::OnNavTwist(double vx, double vy, double omega, bool stamped) {
    // どちらの型で受けているかを最初の1件だけ出す。「Nav2 は動いているのにロボットが
    // 動かない」ときに、配線が繋がっているかを真っ先に切り分けられるようにするため。
    if (stamped && !logged_stamped_) {
        logged_stamped_ = true;
        RCLCPP_INFO(get_logger(), "%s を TwistStamped で受信し始めた", cmd_vel_topic_.c_str());
    } else if (!stamped && !logged_unstamped_) {
        logged_unstamped_ = true;
        RCLCPP_INFO(get_logger(), "%s を Twist(タイムスタンプ無し)で受信し始めた", cmd_vel_topic_.c_str());
    }
    last_cmd_time_ = now();
    warned_no_cmd_ = false;
    mgr_->OnNavTwist(vx, vy, omega);
}

void CmdRouterNode::OnEStop(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        mgr_->EStop();
        RCLCPP_ERROR(get_logger(), "/g1/estop によりE_STOPへ遷移した");
    }
}

void CmdRouterNode::OnTimer() {
    mgr_->Tick();
    WarnIfNoCommand();
    PublishDiagnostics();
}

// NAVIGATING なのに指令が1件も来ない状態を可視化する。**状態は変えない**。
// SafetyManager は「最初の指令が来るまで cmd_timeout の計測を始めない」設計なので
// (Nav2 の計画時間を待つため、意図的)、配線が間違っていると FAULT にも落ちず
// ログも出ないまま静かに止まったままになる。2026-09-13 に Humble の velocity_smoother と
// 繋いだとき、実際にこの沈黙に遭遇した。
void CmdRouterNode::WarnIfNoCommand() {
    const bool navigating = mgr_->state() == g1_sdk_bridge::NavState::kNavigating;
    if (!navigating) {
        navigating_since_.reset();
        last_cmd_time_.reset();
        warned_no_cmd_ = false;
        return;
    }
    if (!navigating_since_.has_value()) {
        navigating_since_ = now();
    }
    if (warned_no_cmd_ || no_cmd_warn_s_ <= 0.0) {
        return;
    }
    // 基準は「最後に指令を受けた時刻」。一度も受けていなければ NAVIGATING に入った時刻。
    // NAVIGATING 開始時刻だけを基準にすると、指令が届き始めた後も警告が出続ける。
    const rclcpp::Time since = last_cmd_time_.value_or(*navigating_since_);
    if ((now() - since).seconds() < no_cmd_warn_s_) {
        return;
    }
    warned_no_cmd_ = true;
    RCLCPP_WARN(get_logger(),
                "NAVIGATING だが %s の指令が %.1f 秒間届いていない%s。"
                "トピック名か**メッセージ型**の食い違いを疑うこと"
                "(`ros2 topic info %s --verbose` で型が2つ並んでいないか確認する)。",
                cmd_vel_topic_.c_str(), no_cmd_warn_s_,
                last_cmd_time_.has_value() ? "" : "(一度も受信していない)",
                cmd_vel_topic_.c_str());
}

void CmdRouterNode::IpcSend(double vx, double vy, double omega) {
    std::lock_guard<std::mutex> lock(ipc_mutex_);
    if (!cmd_endpoint_.has_value()) {
        // 未接続。ReconnectLoop()が繋ぎ直すのを待つ(D-10のROS側watchdogは接続の有無に
        // 関わらず定期的にTick()を回すので、再接続後は次のOnNavTwist/Tickで復旧する)。
        return;
    }
    const CmdPacket pkt{++seq_, MonotonicNs(), vx, vy, omega};
    const CmdWire wire = pkt.Encode();
    try {
        cmd_endpoint_->SendLatest(&wire, sizeof(wire));
    } catch (const PeerClosed&) {
        cmd_endpoint_.reset();
        mgr_->OnBridgeDisconnected();
        RCLCPP_WARN(get_logger(), "SDK側プロセスとの接続が切れた");
    }
}

void CmdRouterNode::ReconnectLoop() {
    while (running_) {
        bool connected;
        {
            std::lock_guard<std::mutex> lock(ipc_mutex_);
            connected = cmd_endpoint_.has_value();
        }
        if (!connected) {
            try {
                // ConnectClient自体が最大2秒、10ms間隔で再試行する(g1_sdk_bridge_cpp/ipc_transport.cpp)。
                auto ep = ConnectClient(cmd_sock_path_, sizeof(CmdWire), 2.0);
                {
                    std::lock_guard<std::mutex> lock(ipc_mutex_);
                    cmd_endpoint_ = std::move(ep);
                }
                mgr_->OnBridgeConnected();
                // TODO(MVP簡易実装): 本来STANDBY->READYは「歩行可能・センサー正常」の確認後に
                // 遷移すべき(仕様書7章)。TF/センサー鮮度チェックをまだ配線していないため、
                // 暫定的にSDK接続=READYとしている。Phase 2c(Nav2接続)着手時に、
                // TFの存在確認・センサーのタイムスタンプ確認を経てからMarkReady()を
                // 呼ぶように置き換えること。
                mgr_->MarkReady();
                RCLCPP_INFO(get_logger(), "SDK側プロセスに接続した: %s", cmd_sock_path_.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "SDK側プロセスへの接続待ち: %s", e.what());
            }
        } else {
            std::this_thread::sleep_for(200ms);
        }
    }
}

void CmdRouterNode::OnEnableNavigation(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                        std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    res->success = mgr_->EnableNavigation(req->data);
    res->message = res->success ? "ok" : "READY状態でないため許可できない(現在の状態を確認すること)";
}

void CmdRouterNode::OnStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    // 仕様書5.2: Nav2キャンセル後、ゼロ速度を即時送信する。Nav2 Goalのキャンセル自体は
    // 上位(mission/bringup層、未実装)の責務。ここではNAVIGATINGを抜けてゼロ速度を送るところまで行う。
    mgr_->EnableNavigation(false);
    res->success = true;
    res->message = "ゼロ速度を送信した";
}

void CmdRouterNode::OnClearFault(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = mgr_->ClearFault();
    // TODO(MVP簡易実装): MarkReady()呼び出しはOnBridgeConnected()と同じ簡略化(本来はTF/センサー
    // 鮮度確認後に遷移すべき、仕様書7章)。ClearFault後にSTANDBYのまま止まらないよう、
    // ここでも同じ簡略化を適用している。Phase 2c着手時に正しい判定へまとめて置き換えること。
    if (res->success) {
        mgr_->MarkReady();
    }
    res->message = res->success ? "ok" : "FAULT状態でないため解除の必要がない";
}

void CmdRouterNode::PublishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "g1_cmd_router";
    st.hardware_id = "g1";
    const NavState state = mgr_->state();
    st.message = NavStateName(state);
    switch (state) {
        case NavState::kNavigating:
        case NavState::kReady:
        case NavState::kStandby:
            st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            break;
        case NavState::kDisconnected:
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            break;
        case NavState::kFault:
        case NavState::kEStop:
            st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            break;
    }
    if (mgr_->fault_reason().has_value()) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "fault_reason";
        kv.value = *mgr_->fault_reason();
        st.values.push_back(kv);
    }
    arr.status.push_back(st);
    pub_diag_->publish(arr);
}

}  // namespace g1_cmd_router
