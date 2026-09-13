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

    // D-23 では TwistStamped に統一する方針だったが、**Nav2 側が出す型はディストリで異なる**
    // (Humble は Twist 固定)。**両方を同時購読すると FastDDS で落ちる**ので、
    // publisher の型を実行時に見て合う方を1本だけ張る。詳細はヘッダのコメント。
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_smoothed");
    no_cmd_warn_s_ = declare_parameter<double>("no_cmd_warn_s", 3.0);
    cmd_vel_type_ = declare_parameter<std::string>("cmd_vel_type", "auto");
    if (cmd_vel_type_ == "twist") {
        CreateUnstampedSubscription();
    } else if (cmd_vel_type_ == "twist_stamped") {
        CreateStampedSubscription();
    } else {
        if (cmd_vel_type_ != "auto") {
            RCLCPP_WARN(get_logger(),
                        "cmd_vel_type='%s' は未知。auto として扱う(twist / twist_stamped / auto)",
                        cmd_vel_type_.c_str());
            cmd_vel_type_ = "auto";
        }
        // publisher が現れるまで購読を作らない。現れたらその型に合わせて1本張る。
        resolve_timer_ = create_wall_timer(500ms, [this]() {
            if (TryCreateCmdVelSubscription()) {
                resolve_timer_->cancel();
            }
        });
    }

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

    // --- D-31: 操作PC 生存監視 ---------------------------------------------
    // 2026-09-09 の実測で、走行中に操作PC のリンクを切ってもロボットは
    // 4.045 秒歩き続け約 0.85m 前進した。既存の watchdog は全て「オンボード側の
    // 誰かが死ぬこと」を見ているので、通信断(オンボード側は全員無事)では効かない。
    heartbeat_required_ = declare_parameter<bool>("heartbeat_required", true);
    const int hb_port = declare_parameter<int>("heartbeat_port", g1_sdk_bridge::kDefaultHeartbeatPort);
    const double hb_timeout = declare_parameter<double>("operator_timeout_s", 1.0);
    const std::string hb_bind = declare_parameter<std::string>("heartbeat_bind_address", "");
    if (heartbeat_required_) {
        // ⚠️ ここで例外が出たら**起動を失敗させる**。監視できないまま
        // 「監視しているつもり」で走り出すのが一番危ない。
        heartbeat_ = std::make_unique<g1_sdk_bridge::HeartbeatReceiver>(
            static_cast<std::uint16_t>(hb_port), hb_timeout, hb_bind);
        heartbeat_->Start();
        RCLCPP_INFO(get_logger(),
                    "操作PCの生存監視を開始した(UDP port=%d, timeout=%.2fs)。"
                    "操作PC側で g1_heartbeat_sender を動かすこと",
                    hb_port, hb_timeout);
    } else {
        RCLCPP_WARN(get_logger(),
                    "⚠️ 操作PCの生存監視が無効(heartbeat_required=false)。"
                    "通信断でロボットは止まらない(D-31)。ベンチ試験以外で使わないこと");
    }

    // --- Nav2 Goal のキャンセル -------------------------------------------
    // FAULT で指令の転送は止まるが、`bt_navigator` の Goal は生きたままなので、
    // そのままだと `clear_fault` した瞬間に中断地点から巡回が再開してしまう。
    // 人は「復帰させた」だけのつもりなので、これは驚きが大きい。
    nav2_cancel_services_ = declare_parameter<std::vector<std::string>>(
        "nav2_cancel_services",
        std::vector<std::string>{"/navigate_to_pose/_action/cancel_goal",
                                 "/navigate_through_poses/_action/cancel_goal"});
    for (const auto& name : nav2_cancel_services_) {
        if (!name.empty()) {
            cancel_clients_.push_back(create_client<action_msgs::srv::CancelGoal>(name));
        }
    }
    if (cancel_clients_.empty()) {
        RCLCPP_WARN(get_logger(), "Nav2 Goal のキャンセルが無効。停止後に巡回が再開しうる");
    }

    reconnect_thread_ = std::thread(&CmdRouterNode::ReconnectLoop, this);

    RCLCPP_INFO(get_logger(), "g1_cmd_router 起動。cmd_sock_path=%s", cmd_sock_path_.c_str());
}

CmdRouterNode::~CmdRouterNode() {
    if (heartbeat_) {
        heartbeat_->Stop();
    }
    running_ = false;
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
}

void CmdRouterNode::CreateStampedSubscription() {
    sub_twist_stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, 10,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) { OnTwistStamped(msg); });
    RCLCPP_INFO(get_logger(), "%s を TwistStamped として購読する", cmd_vel_topic_.c_str());
}

void CmdRouterNode::CreateUnstampedSubscription() {
    sub_twist_unstamped_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) { OnTwistUnstamped(msg); });
    RCLCPP_INFO(get_logger(), "%s を Twist(タイムスタンプ無し)として購読する", cmd_vel_topic_.c_str());
}

// publisher が出している型を調べ、合う購読を1本だけ作る。
// ⚠️ **両方を同時に張ると FastDDS が例外を投げてノードごと落ちる**(ヘッダのコメント参照)。
bool CmdRouterNode::TryCreateCmdVelSubscription() {
    const auto infos = get_publishers_info_by_topic(cmd_vel_topic_);
    if (infos.empty()) {
        return false;  // まだ Nav2 が上がっていない。次の tick で見直す
    }
    bool has_stamped = false;
    bool has_plain = false;
    for (const auto& info : infos) {
        if (info.topic_type() == "geometry_msgs/msg/TwistStamped") has_stamped = true;
        if (info.topic_type() == "geometry_msgs/msg/Twist") has_plain = true;
    }
    if (has_stamped && has_plain) {
        // 両方の publisher が居る構成は想定していない。片方しか受けられないので、
        // 黙って選ばずに warn する(どちらを選んでも半分の指令を取りこぼす)。
        RCLCPP_WARN(get_logger(),
                    "%s に Twist と TwistStamped の publisher が同時に居る。"
                    "TwistStamped を選ぶが、構成を見直すこと",
                    cmd_vel_topic_.c_str());
    }
    if (has_stamped) {
        CreateStampedSubscription();
        return true;
    }
    if (has_plain) {
        CreateUnstampedSubscription();
        return true;
    }
    RCLCPP_WARN(get_logger(), "%s の publisher の型が想定外: %s",
                cmd_vel_topic_.c_str(), infos.front().topic_type().c_str());
    return false;
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
    CheckOperatorHeartbeat();
    WarnIfNoCommand();

    // NAVIGATING から異常系へ抜けたら Nav2 の Goal も取り消す。
    // 個々の異常(cmd_timeout / operator_lost / sdk_bridge_error / E-stop)ごとに
    // 呼び出しを散らさず、**状態遷移という1箇所で拾う**。呼び忘れが起きないため。
    const auto state = mgr_->state();
    if (prev_state_ == g1_sdk_bridge::NavState::kNavigating &&
        (state == g1_sdk_bridge::NavState::kFault || state == g1_sdk_bridge::NavState::kEStop)) {
        CancelNav2Goals(mgr_->fault_reason().value_or(
            state == g1_sdk_bridge::NavState::kEStop ? "e_stop" : "fault"));
    }
    prev_state_ = state;

    PublishDiagnostics();
}

// Nav2 の実行中 Goal を取り消す(best-effort)。詳細はヘッダのコメント。
void CmdRouterNode::CancelNav2Goals(const std::string& reason) {
    for (const auto& client : cancel_clients_) {
        if (!client->service_is_ready()) {
            // Nav2 が上がっていない/既に落ちている。停止処理自体は続ける。
            RCLCPP_WARN(get_logger(), "%s が応答しないため Goal を取り消せない(理由=%s)",
                        client->get_service_name(), reason.c_str());
            continue;
        }
        // goal_id と stamp を 0 のままにすると「**全ての Goal を取り消す**」という
        // action_msgs/srv/CancelGoal の規約になる。どの Goal が走っているかを
        // 追跡しなくてよいので、取りこぼしが起きない。
        auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
        const std::string service_name = client->get_service_name();
        // ⚠️ 同期的に待たない。ここは 50ms 周期のタイマーとサービスコールバックの
        // 中から呼ばれるので、応答待ちで実行器を止めると停止処理ごと固まる。
        client->async_send_request(
            req, [this, service_name](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture fut) {
                const auto res = fut.get();
                using Resp = action_msgs::srv::CancelGoal::Response;
                if (res->return_code == Resp::ERROR_NONE) {
                    RCLCPP_INFO(get_logger(), "Nav2 Goal を %zu 件取り消した(%s)",
                                res->goals_canceling.size(), service_name.c_str());
                } else if (res->return_code == Resp::ERROR_GOAL_TERMINATED ||
                           res->goals_canceling.empty()) {
                    // 走っている Goal が無かっただけ。異常ではない
                    RCLCPP_INFO(get_logger(), "取り消す Nav2 Goal は無かった(%s)", service_name.c_str());
                } else {
                    RCLCPP_WARN(get_logger(), "Nav2 Goal の取り消しが拒否された: return_code=%d (%s)",
                                static_cast<int>(res->return_code), service_name.c_str());
                }
            });
        RCLCPP_INFO(get_logger(), "Nav2 Goal の取り消しを要求した(理由=%s, %s)", reason.c_str(),
                    service_name.c_str());
    }
}

// D-31: 操作PC との通信が途絶していたら FAULT へ落とす。
//
// ⚠️ **これは停止手段ではない。** 被害を小さくする仕組みであって、
// 物理的な停止手段(純正リモコン)が唯一の最終防衛線であることは変わらない(§7)。
void CmdRouterNode::CheckOperatorHeartbeat() {
    if (!heartbeat_) {
        return;
    }
    // NAVIGATING 以外では監視しない。待機中に操作PC を落としただけで
    // FAULT にすると、起動手順の順番に過剰な制約がかかるため。
    if (mgr_->state() != g1_sdk_bridge::NavState::kNavigating) {
        warned_heartbeat_missing_ = false;
        return;
    }
    if (heartbeat_->Alive()) {
        warned_heartbeat_missing_ = false;
        return;
    }
    if (warned_heartbeat_missing_) {
        return;  // 既に FAULT にした。毎 tick ログを出さない
    }
    warned_heartbeat_missing_ = true;
    const auto since = heartbeat_->SecondsSinceLast();
    // 一時オブジェクトの c_str() を三項演算子の中で渡すと寿命が読みにくいので、
    // 名前を付けてから渡す。
    const std::string since_text =
        since.has_value() ? (std::to_string(*since) + "秒") : std::string("一度も受信していない");
    RCLCPP_ERROR(get_logger(),
                 "操作PCのheartbeatが途絶した(最終受信から%s)。巡回を停止する(D-31)。"
                 "復帰には操作PC側の送信再開と /g1/clear_fault が必要",
                 since_text.c_str());
    mgr_->OnOperatorLost();
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
    // D-31: heartbeat を一度も受けていない状態で走り出させない。
    // 「操作PC側の送信を立ち上げ忘れたまま巡回を始めてしまう」を防ぐ。
    // ⚠️ 有効化を拒むのは NAVIGATING に入るときだけ。無効化(false)は常に通す。
    if (req->data && heartbeat_ && !heartbeat_->Alive()) {
        res->success = false;
        res->message = heartbeat_->EverReceived()
                           ? "操作PCのheartbeatが途絶している(D-31)。送信を再開してから有効化すること"
                           : "操作PCのheartbeatを一度も受信していない(D-31)。"
                             "操作PC側で g1_heartbeat_sender を起動すること";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
    }
    res->success = mgr_->EnableNavigation(req->data);
    res->message = res->success ? "ok" : "READY状態でないため許可できない(現在の状態を確認すること)";
}

void CmdRouterNode::OnStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    // 仕様書5.2: Nav2キャンセル後、ゼロ速度を即時送信する。
    // ⚠️ **順序が逆に見えるが、ゼロ速度を先に送るのが正しい。**
    // キャンセルは非同期で、応答を待つ間もロボットは歩いている。
    // 止めることを最優先し、Goal の取り消しはその後で要求する。
    mgr_->EnableNavigation(false);
    CancelNav2Goals("stop_service");
    res->success = true;
    res->message = "ゼロ速度を送信し、Nav2 Goal の取り消しを要求した";
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
    // D-31: 操作PC の生存状況も診断に載せる。「なぜ止まったのか」を
    // rosbag から追跡できるようにするため(Phase 2c 完了条件)。
    {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "operator_heartbeat";
        if (!heartbeat_) {
            kv.value = "disabled";
        } else if (!heartbeat_->EverReceived()) {
            kv.value = "never_received";
        } else {
            kv.value = heartbeat_->Alive() ? "alive" : "lost";
        }
        st.values.push_back(kv);
    }
    if (heartbeat_) {
        const auto since = heartbeat_->SecondsSinceLast();
        if (since.has_value()) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = "operator_heartbeat_age_s";
            kv.value = std::to_string(*since);
            st.values.push_back(kv);
        }
    }
    arr.status.push_back(st);
    pub_diag_->publish(arr);
}

}  // namespace g1_cmd_router
