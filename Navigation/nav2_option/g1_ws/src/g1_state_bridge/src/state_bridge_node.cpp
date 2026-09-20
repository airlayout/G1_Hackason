#include "g1_state_bridge/state_bridge_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "g1_sdk_bridge/protocol.hpp"

using namespace std::chrono_literals;
using g1_sdk_bridge::BridgeStatus;
using g1_sdk_bridge::ConnectClient;
using g1_sdk_bridge::PeerClosed;
using g1_sdk_bridge::StatePacket;
using g1_sdk_bridge::StateWire;

namespace g1_state_bridge {

namespace {
const char* StatusName(BridgeStatus s) {
    switch (s) {
        case BridgeStatus::kDisconnected: return "DISCONNECTED";
        case BridgeStatus::kStandby: return "STANDBY";
        case BridgeStatus::kReady: return "READY";
        case BridgeStatus::kNavigating: return "NAVIGATING";
        case BridgeStatus::kFault: return "FAULT";
    }
    return "UNKNOWN";
}
}  // namespace

StateBridgeNode::StateBridgeNode() : rclcpp::Node("g1_state_bridge") {
    state_sock_path_ = declare_parameter<std::string>("state_sock_path", "/tmp/g1_bridge/state.sock");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");

    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/g1/state_bridge_status", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 20Hz程度で読み取る(SDK側の状態配信周期に合わせる。仕様書8章cmd_rateの既定値と同程度)。
    timer_ = create_wall_timer(50ms, [this]() { OnTimer(); });

    reconnect_thread_ = std::thread(&StateBridgeNode::ReconnectLoop, this);

    RCLCPP_INFO(get_logger(), "g1_state_bridge 起動。state_sock_path=%s", state_sock_path_.c_str());
}

StateBridgeNode::~StateBridgeNode() {
    running_ = false;
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
}

void StateBridgeNode::ReconnectLoop() {
    while (running_) {
        bool connected;
        {
            std::lock_guard<std::mutex> lock(ipc_mutex_);
            connected = state_endpoint_.has_value();
        }
        if (!connected) {
            try {
                auto ep = ConnectClient(state_sock_path_, sizeof(StateWire), 2.0);
                std::lock_guard<std::mutex> lock(ipc_mutex_);
                state_endpoint_ = std::move(ep);
                RCLCPP_INFO(get_logger(), "SDK側プロセスのstateソケットに接続した");
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "stateソケットへの接続待ち: %s", e.what());
            }
        } else {
            std::this_thread::sleep_for(200ms);
        }
    }
}

void StateBridgeNode::OnTimer() {
    std::optional<std::vector<std::uint8_t>> raw;
    bool disconnected_now = false;
    {
        std::lock_guard<std::mutex> lock(ipc_mutex_);
        if (state_endpoint_.has_value()) {
            try {
                raw = state_endpoint_->RecvLatest();
            } catch (const PeerClosed&) {
                state_endpoint_.reset();
                disconnected_now = true;
            }
        }
    }
    if (disconnected_now) {
        RCLCPP_WARN(get_logger(), "stateソケットの接続が切れた");
    }
    if (!raw.has_value()) {
        return;
    }

    StatePacket pkt;
    try {
        pkt = StatePacket::Decode(raw->data(), raw->size());
    } catch (const g1_sdk_bridge::ProtocolError&) {
        return;  // 破損パケットは無視
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = pkt.x;
    odom.pose.pose.position.y = pkt.y;
    odom.pose.pose.position.z = 0.0;
    // yawのみの回転(ピッチ・ロールは扱わない、D-19の簡易積分に対応する範囲)
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(pkt.yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(pkt.yaw / 2.0);
    odom.twist.twist.linear.x = pkt.vx;
    odom.twist.twist.linear.y = pkt.vy;
    odom.twist.twist.angular.z = pkt.omega;
    pub_odom_->publish(odom);

    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = odom.header.stamp;
        tf.header.frame_id = odom_frame_id_;
        tf.child_frame_id = base_frame_id_;
        tf.transform.translation.x = pkt.x;
        tf.transform.translation.y = pkt.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "g1_state_bridge";
    st.hardware_id = "g1";
    st.message = StatusName(pkt.status);
    st.level = (pkt.status == BridgeStatus::kFault) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
               : (pkt.status == BridgeStatus::kDisconnected) ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                                              : diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "sdk_error_count";
    kv.value = std::to_string(pkt.sdk_error_count);
    st.values.push_back(kv);
    arr.status.push_back(st);
    pub_diag_->publish(arr);
}

}  // namespace g1_state_bridge
