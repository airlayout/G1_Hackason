// Phase 1 用の teleop クライアント。ROS を介さず IPC 直結で SDK 側プロセスを動かす。
//
// ## なぜ ROS を使わないのか
//
// IPC は Unix domain socket なので、ROS 側(`g1_cmd_router`)は SDK 側プロセスと**同一ホスト**
// で動く必要がある。しかし PC2 に入っているのは ROS 2 Foxy で、`g1_ws` は Jazzy 向け(D-01)。
// 環境を揃える前に、**まず SDK 側プロセス単体を実機で検証する**のがこのクライアントの役目。
// 検証できるのは D-09(20Hz周期送信) / D-10(watchdog) / D-11(起動時ゼロ) / RealMoveBackend。
//
// ## 暴走しない作り
//
// **対話的なキー入力にはしていない。** キーが押されたままになる/端末が切れる、といった形で
// 非ゼロ速度が出続ける事故を避けるため、**`--seconds` で指定した時間だけ送り、その後必ず
// ゼロ速度を送ってから終了する**。上限は `--seconds` 自体でクランプする。
//
// ## 使い方
//
//     # ① 素振り(SDK側を --arm 無しで起動しておけば機体は動かない)
//     g1_sdk_bridge_teleop --vx 0.3 --seconds 2
//
//     # ② watchdog の確認: 送信を途中で止める(SIGINT)と SDK 側がゼロを出すはず
//     g1_sdk_bridge_teleop --vx 0.3 --seconds 30    (途中で Ctrl-C)
//
// ⚠️ **実測(U-12)では 0.2 m/s では進行方向が定まらない。0.3 m/s 以上を使うこと。**
// ⚠️ **指令を止めても歩容が終わるまで約2秒動き続ける(U-07)。** 前方の空間を確保すること。

#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "g1_sdk_bridge/ipc_transport.hpp"
#include "g1_sdk_bridge/protocol.hpp"

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
        "  --vx <m/s>        前進速度 (既定 0.0)。**0.3 以上を推奨(U-12)**\n"
        "  --vy <m/s>        横移動速度 (既定 0.0)。MVP では 0(D-15)\n"
        "  --omega <rad/s>   旋回速度 (既定 0.0)\n"
        "  --seconds <s>     非ゼロ速度を送り続ける秒数 (既定 1.0)\n"
        "  --rate <hz>       送信レート (既定 20.0、D-09)\n"
        "  --cmd-sock <PATH>   (既定 /tmp/g1_bridge/cmd.sock)\n"
        "  --state-sock <PATH> (既定 /tmp/g1_bridge/state.sock)\n"
        "\n"
        "送信後は必ずゼロ速度を1秒送ってから終了する。Ctrl-C でも同じ。\n",
        argv0);
}
}  // namespace

int main(int argc, char** argv) {
    double vx = 0.0, vy = 0.0, omega = 0.0, seconds = 1.0, rate = 20.0;
    std::string cmd_path = "/tmp/g1_bridge/cmd.sock";
    std::string state_path = "/tmp/g1_bridge/state.sock";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "[teleop] %s に値がない\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--vx") vx = std::stod(next("--vx"));
        else if (arg == "--vy") vy = std::stod(next("--vy"));
        else if (arg == "--omega") omega = std::stod(next("--omega"));
        else if (arg == "--seconds") seconds = std::stod(next("--seconds"));
        else if (arg == "--rate") rate = std::stod(next("--rate"));
        else if (arg == "--cmd-sock") cmd_path = next("--cmd-sock");
        else if (arg == "--state-sock") state_path = next("--state-sock");
        else if (arg == "--help" || arg == "-h") { PrintUsage(argv[0]); return 0; }
        else { std::fprintf(stderr, "[teleop] 未知の引数: %s\n", arg.c_str()); PrintUsage(argv[0]); return 2; }
    }

    if (seconds <= 0.0 || seconds > 60.0) {
        std::fprintf(stderr, "[teleop] --seconds は 0 より大きく 60 以下にすること(暴走防止)\n");
        return 2;
    }
    if (rate <= 0.0 || rate > 200.0) {
        std::fprintf(stderr, "[teleop] --rate が不正\n");
        return 2;
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::printf("[teleop] vx=%.3f vy=%.3f omega=%.3f を %.1f 秒間 %.0fHz で送る\n", vx, vy, omega, seconds, rate);
    std::printf("[teleop] その後ゼロ速度を1秒送って終了する\n");

    try {
        auto cmd = g1_sdk_bridge::ConnectClient(cmd_path, sizeof(g1_sdk_bridge::CmdWire), 5.0);
        auto state = g1_sdk_bridge::ConnectClient(state_path, sizeof(g1_sdk_bridge::StateWire), 5.0);
        std::printf("[teleop] 接続した cmd=%s state=%s\n", cmd_path.c_str(), state_path.c_str());

        const auto period = std::chrono::duration<double>(1.0 / rate);
        std::uint64_t seq = 0;
        const auto t0 = std::chrono::steady_clock::now();
        auto last_print = t0;

        // フェーズ1: 指定速度を送る / フェーズ2: ゼロを1秒送る
        for (int phase = 0; phase < 2; ++phase) {
            const double dur = (phase == 0) ? seconds : 1.0;
            const double cvx = (phase == 0 && !g_stop) ? vx : 0.0;
            const double cvy = (phase == 0 && !g_stop) ? vy : 0.0;
            const double com = (phase == 0 && !g_stop) ? omega : 0.0;
            if (phase == 1) std::printf("[teleop] ゼロ速度に切り替える\n");

            const auto phase_start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - phase_start < std::chrono::duration<double>(dur)) {
                if (g_stop && phase == 0) break;  // 中断されたらすぐゼロ送信フェーズへ
                // 一時オブジェクトのアドレスは取れないので名前を付ける
                const auto wire = g1_sdk_bridge::MakeCmd(++seq, cvx, cvy, com).Encode();
                cmd.SendLatest(&wire, sizeof(wire));

                if (auto msg = state.RecvLatest()) {
                    const auto st = g1_sdk_bridge::StatePacket::Decode(msg->data(), msg->size());
                    const auto now = std::chrono::steady_clock::now();
                    if (now - last_print > std::chrono::milliseconds(400)) {
                        std::printf("[teleop] t=%5.2fs status=%-11s 指令(v=%.3f,%.3f,%.3f) "
                                    "推定pose(%.3f,%.3f,%.1f°) sdk_err=%u\n",
                                    std::chrono::duration<double>(now - t0).count(), StatusName(st.status),
                                    st.vx, st.vy, st.omega, st.x, st.y, st.yaw * 57.2958, st.sdk_error_count);
                        last_print = now;
                    }
                }
                std::this_thread::sleep_for(period);
            }
        }
        std::printf("[teleop] 終了する(ゼロ速度を送り終えた)\n");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[teleop] エラー: %s\n", e.what());
        std::fprintf(stderr, "         SDK側プロセス(g1_sdk_bridge_real_server)が起動しているか確認すること\n");
        return 1;
    }
    return 0;
}
