// SDK側プロセスの**本番**実行ファイル。unitree_sdk2 の LocoClient を叩く。
//
// Planning.md の D-04〜D-11 に対応する常駐プロセス。ROS 側(g1_cmd_router)から
// Unix domain socket で速度指令を受け、20Hz で G1 へ SetVelocity を送る。
// watchdog・起動時ゼロ速度・FAULT 遷移は SdkBridgeProcess 側が持つ。
//
// ## 使い方
//
//     # ① 素振り(発進ゲート閉)。SDK を呼ばずに IPC と状態機械だけ確認する
//     g1_sdk_bridge_real_server --network-interface eth0
//
//     # ② 実機を動かす。**人が支え、停止手段を持った状態で**
//     g1_sdk_bridge_real_server --network-interface eth0 --arm
//
// `--arm` を付けない限り SDK は一切呼ばれない(RealMoveBackend の発進ゲート)。
//
// ⚠️ **ROS 環境を source した状態で起動しないこと**(D-07)。`LD_LIBRARY_PATH` /
// `AMENT_PREFIX_PATH` を継承すると ROS 側の CycloneDDS に誤リンクする。
// systemd から起動する場合も環境を継承させない。

#include <csignal>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include "g1_sdk_bridge/protocol.hpp"
#include "g1_sdk_bridge/real_move_backend.hpp"
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

void PrintUsage(const char* argv0) {
    std::printf(
        "使い方: %s [オプション]\n"
        "  --network-interface <IF>  G1内蔵スイッチ側のIF名 (既定: eth0)\n"
        "  --domain-id <N>           DDS domain id (既定: 0)\n"
        "  --cmd-sock <PATH>         cmd用ソケット (既定: /tmp/g1_bridge/cmd.sock)\n"
        "  --state-sock <PATH>       state用ソケット (既定: /tmp/g1_bridge/state.sock)\n"
        "  --arm                     発進を許可する。付けなければSDKを一切呼ばない\n"
        "  --help                    この表示\n",
        argv0);
}
}  // namespace

int main(int argc, char** argv) {
    g1_sdk_bridge::RealMoveBackendConfig backend_cfg;
    std::string cmd_path = "/tmp/g1_bridge/cmd.sock";
    std::string state_path = "/tmp/g1_bridge/state.sock";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "[real_server] %s に値がない\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--network-interface") {
            backend_cfg.network_interface = next("--network-interface");
        } else if (arg == "--domain-id") {
            backend_cfg.domain_id = std::stoi(next("--domain-id"));
        } else if (arg == "--cmd-sock") {
            cmd_path = next("--cmd-sock");
        } else if (arg == "--state-sock") {
            state_path = next("--state-sock");
        } else if (arg == "--arm") {
            backend_cfg.armed = true;
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "[real_server] 未知の引数: %s\n", arg.c_str());
            PrintUsage(argv[0]);
            return 2;
        }
    }

    std::filesystem::create_directories(std::filesystem::path(cmd_path).parent_path());
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    g1_sdk_bridge::SdkBridgeConfig cfg;
    cfg.cmd_sock_path = cmd_path;
    cfg.state_sock_path = state_path;
    // 発進ゲートの手前でも duration の上限を SdkBridgeConfig と揃えておく
    backend_cfg.max_duration_s = cfg.cmd_timeout_s;

    if (!backend_cfg.armed) {
        std::printf(
            "[real_server] ⚠️ 発進ゲートが閉じています(--arm 無し)。\n"
            "              IPC と状態機械は動きますが、SDK へは一切送りません。\n");
    } else {
        std::printf(
            "[real_server] ⚠️⚠️ 発進ゲートが開いています(--arm)。**G1 が実際に動きます。**\n"
            "              人が支え、純正リモコンで停止できる状態であることを確認してください。\n");
    }

    try {
        g1_sdk_bridge::RealMoveBackend backend(backend_cfg);
        g1_sdk_bridge::SdkBridgeProcess bridge(cfg, backend);
        bridge.Start();
        std::printf("[real_server] 起動した。cmd=%s state=%s\n", cmd_path.c_str(), state_path.c_str());

        auto last_status = static_cast<g1_sdk_bridge::BridgeStatus>(-1);
        std::size_t last_sent = 0, last_blocked = 0;
        while (!g_stop) {
            const auto status = bridge.status();
            if (status != last_status) {
                std::printf("[real_server] status -> %s\n", StatusName(status));
                last_status = status;
            }
            const auto sent = backend.sent_count();
            const auto blocked = backend.blocked_count();
            if (sent != last_sent || blocked != last_blocked) {
                std::printf("[real_server] SDK送信 %zu 件 / ゲートで停止 %zu 件\n", sent, blocked);
                last_sent = sent;
                last_blocked = blocked;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        std::printf("[real_server] 終了する\n");
        bridge.Stop();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[real_server] 致命的エラー: %s\n", e.what());
        return 1;
    }
    return 0;
}
