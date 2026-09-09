// SDK側プロセスの動作確認・開発用スタンドアロン実行ファイル。
//
// 実際のG1 SDK2は無いので MockMoveBackend で代替し、g1_cmd_router / g1_state_bridge
// (ROS側)とのIPC疎通をエンドツーエンドで確認するために使う。実機投入時は
// unitree_sdk2 の LocoClient::SetVelocity() を呼ぶ RealMoveBackend に差し替えた
// 別実行ファイルに置き換わる想定(このファイル自体は本番では使わない)。
//
// 使い方: g1_sdk_bridge_mock_server [cmd_sock_path] [state_sock_path]

#include <csignal>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <thread>

#include "g1_sdk_bridge/protocol.hpp"
#include "g1_sdk_bridge/sdk_bridge_process.hpp"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void OnSignal(int) { g_stop = 1; }

const char* StatusName(g1_sdk_bridge::BridgeStatus s) {
    using g1_sdk_bridge::BridgeStatus;
    switch (s) {
        case BridgeStatus::kDisconnected: return "DISCONNECTED";
        case BridgeStatus::kStandby: return "STANDBY";
        case BridgeStatus::kReady: return "READY";
        case BridgeStatus::kNavigating: return "NAVIGATING";
        case BridgeStatus::kFault: return "FAULT";
    }
    return "?";
}
}  // namespace

int main(int argc, char** argv) {
    const std::string cmd_path = argc > 1 ? argv[1] : "/tmp/g1_bridge/cmd.sock";
    const std::string state_path = argc > 2 ? argv[2] : "/tmp/g1_bridge/state.sock";
    std::filesystem::create_directories(std::filesystem::path(cmd_path).parent_path());

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    g1_sdk_bridge::MockMoveBackend backend;
    g1_sdk_bridge::SdkBridgeConfig cfg;
    cfg.cmd_sock_path = cmd_path;
    cfg.state_sock_path = state_path;

    g1_sdk_bridge::SdkBridgeProcess bridge(cfg, backend);
    bridge.Start();
    std::cout << "[mock_server] 起動した。cmd=" << cmd_path << " state=" << state_path << std::endl;

    auto last_status = static_cast<g1_sdk_bridge::BridgeStatus>(-1);
    while (!g_stop) {
        const auto status = bridge.status();
        if (status != last_status) {
            std::cout << "[mock_server] status -> " << StatusName(status) << std::endl;
            last_status = status;
        }
        const auto call = backend.LastCall();
        if (call.has_value()) {
            std::printf("[mock_server] last SetVelocity: vx=%.3f vy=%.3f omega=%.3f duration=%.3f\n", call->vx,
                        call->vy, call->omega, call->duration);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "[mock_server] 終了する" << std::endl;
    bridge.Stop();
    return 0;
}
