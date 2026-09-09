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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "g1_sdk_bridge/ipc_transport.hpp"
#include "g1_sdk_bridge/safety_manager.hpp"

namespace g1_cmd_router {

class CmdRouterNode : public rclcpp::Node {
public:
    CmdRouterNode();
    ~CmdRouterNode() override;

private:
    void OnTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void OnEStop(const std_msgs::msg::Bool::SharedPtr msg);
    void OnTimer();
    void IpcSend(double vx, double vy, double omega);
    void ReconnectLoop();
    void PublishDiagnostics();

    void OnEnableNavigation(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void OnStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void OnClearFault(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    std::string cmd_sock_path_;
    g1_sdk_bridge::SafetyLimits limits_;
    std::unique_ptr<g1_sdk_bridge::SafetyManager> mgr_;

    std::mutex ipc_mutex_;
    std::optional<g1_sdk_bridge::SeqPacketEndpoint> cmd_endpoint_;  // ipc_mutex_で保護
    std::uint64_t seq_ = 0;                                        // ipc_mutex_で保護

    std::atomic<bool> running_{true};
    std::thread reconnect_thread_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_fault_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
};

}  // namespace g1_cmd_router
