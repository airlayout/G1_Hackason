// g1_state_bridge: SDK側プロセスのstate socketから読んだStatePacketを
// nav_msgs/OdometryとDiagnosticArrayに変換して配信するノード(仕様書5.1)。
//
// 注意(D-19): ここで配信する x, y, yaw は「SDK側プロセスが供給した値」をそのまま
// 中継しているに過ぎない。G1のSDK2にはodometryの情報源が無いため(sdk2_api.md §6)、
// 実機では別途LIOから供給されるodometryをSDK側プロセスが受け取り、IPCで送ってくる
// 実装が必要になる(現時点のsdk_bridge_processはデモ用の簡易積分)。

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "g1_sdk_bridge/ipc_transport.hpp"

namespace g1_state_bridge {

class StateBridgeNode : public rclcpp::Node {
public:
    StateBridgeNode();
    ~StateBridgeNode() override;

private:
    void OnTimer();
    void ReconnectLoop();

    std::string state_sock_path_;
    std::string odom_frame_id_;
    std::string base_frame_id_;

    std::mutex ipc_mutex_;
    std::optional<g1_sdk_bridge::SeqPacketEndpoint> state_endpoint_;  // ipc_mutex_で保護

    std::atomic<bool> running_{true};
    std::thread reconnect_thread_;

    bool publish_tf_ = true;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace g1_state_bridge
