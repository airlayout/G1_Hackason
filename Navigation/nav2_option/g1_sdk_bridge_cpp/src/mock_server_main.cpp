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
#include <string>
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
    std::string cmd_path = "/tmp/g1_bridge/cmd.sock";
    std::string state_path = "/tmp/g1_bridge/state.sock";
    // ⚠️ **二足の最小作動閾値を模す。** 既定は 0(無効)＝指令どおりに完璧に動く。
    // 値を入れると「指令が小さすぎると動かない」実機の性質が入り、
    // 2026-09-15 に踏んだ旋回デッドロックを実機なしで再現できる。
    // 実測の目安: vx は 0.2 未満で歩容が成立しない、omega は 0.02 で不動・0.3 で回る。
    double gait_min_vx = 0.0;
    double gait_min_omega = 0.0;
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--gait-min-vx" && i + 1 < argc) {
            gait_min_vx = std::stod(argv[++i]);
        } else if (a == "--gait-min-omega" && i + 1 < argc) {
            gait_min_omega = std::stod(argv[++i]);
        } else if (a == "--like-g1") {
            // 実機に近い閾値をまとめて入れる近道
            gait_min_vx = 0.2;
            gait_min_omega = 0.15;
        } else if (a == "--help" || a == "-h") {
            std::cout << "使い方: g1_sdk_bridge_mock_server [cmd_sock] [state_sock]\n"
                      << "  --gait-min-vx <m/s>      これ未満の並進指令では動かない(既定 0=無効)\n"
                      << "  --gait-min-omega <rad/s> これ未満の旋回指令では動かない(既定 0=無効)\n"
                      << "  --like-g1                実機に近い閾値(vx 0.2 / omega 0.15)をまとめて入れる\n";
            return 0;
        } else if (positional == 0) {
            cmd_path = a;
            ++positional;
        } else if (positional == 1) {
            state_path = a;
            ++positional;
        }
    }
    std::filesystem::create_directories(std::filesystem::path(cmd_path).parent_path());

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    g1_sdk_bridge::MockMoveBackend backend;
    g1_sdk_bridge::SdkBridgeConfig cfg;
    cfg.cmd_sock_path = cmd_path;
    cfg.state_sock_path = state_path;
    cfg.gait_min_vx = gait_min_vx;
    cfg.gait_min_omega = gait_min_omega;
    if (gait_min_vx > 0.0 || gait_min_omega > 0.0) {
        std::cout << "[mock_server] 最小作動閾値: vx=" << gait_min_vx
                  << " omega=" << gait_min_omega << " (これ未満の指令では動かない)" << std::endl;
    }

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
