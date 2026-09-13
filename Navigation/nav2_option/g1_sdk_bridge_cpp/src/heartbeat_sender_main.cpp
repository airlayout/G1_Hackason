// 操作PC 側で動かす heartbeat 送信プログラム。Planning.md **D-31**。
// ROS を一切使わないので、ROS の入っていない操作PC でもそのまま動く。
//
// ## これは何か
//
// **「操作PC がまだ生きている」ことだけを PC2 に伝え続けるプログラム。**
// 速度指令は一切送らない。**このプログラムから機体を動かすことはできない。**
//
// PC2 側の `g1_cmd_router` はこれが途絶えると巡回を止める(D-31)。
// 2026-09-09 の実測で、通信断でもロボットが 4.045 秒・0.85m 進み続けたため。
//
// ## 使い方
//
//     # PC2 の IP へ 5Hz で送り続ける。Ctrl-C で止める
//     g1_heartbeat_sender --host 192.168.123.222
//
// ⚠️ **止めるとロボットが止まる。** 巡回中に何気なく Ctrl-C しないこと。
// ⚠️ このプログラムを動かしたまま操作PC がスリープすると、途絶と見なされる。
//
// ## 停止手段ではない
//
// **これは被害を小さくする仕組みであって、停止手段ではない。**
// 物理的な停止手段(純正リモコン)が唯一の最終防衛線であることは変わらない
// (Planning.md §7)。

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <thread>

#include "g1_sdk_bridge/heartbeat.hpp"
#include "g1_sdk_bridge/protocol.hpp"  // MonotonicNs

namespace {
volatile std::sig_atomic_t g_stop = 0;
void OnSignal(int) { g_stop = 1; }

void PrintUsage(const char* argv0) {
    std::printf(
        "使い方: %s --host <PC2のIP> [オプション]\n"
        "  --host <IP>     送信先(PC2)のIPアドレス。**必須**\n"
        "  --port <N>      送信先ポート (既定 %u)\n"
        "  --rate <hz>     送信レート (既定 5.0)\n"
        "  --help          この表示\n"
        "\n"
        "⚠️ これは「操作PCが生きている」ことだけを伝えるプログラムで、\n"
        "   速度指令は一切送らない。**止めるとロボットが止まる**(D-31)。\n",
        argv0, static_cast<unsigned>(g1_sdk_bridge::kDefaultHeartbeatPort));
}
}  // namespace

int main(int argc, char** argv) {
    std::string host;
    std::uint16_t port = g1_sdk_bridge::kDefaultHeartbeatPort;
    double rate = 5.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "[heartbeat] %s に値がない\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--host") host = next("--host");
        else if (arg == "--port") port = static_cast<std::uint16_t>(std::stoi(next("--port")));
        else if (arg == "--rate") rate = std::stod(next("--rate"));
        else if (arg == "--help" || arg == "-h") { PrintUsage(argv[0]); return 0; }
        else { std::fprintf(stderr, "[heartbeat] 未知の引数: %s\n", arg.c_str()); PrintUsage(argv[0]); return 2; }
    }
    if (host.empty()) {
        std::fprintf(stderr, "[heartbeat] --host は必須\n");
        PrintUsage(argv[0]);
        return 2;
    }
    if (rate <= 0.0 || rate > 100.0) {
        std::fprintf(stderr, "[heartbeat] --rate が不正(0 より大きく 100 以下)\n");
        return 2;
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::fprintf(stderr, "[heartbeat] UDPソケットを作れない: %s\n", std::strerror(errno));
        return 1;
    }
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &dst.sin_addr) != 1) {
        std::fprintf(stderr, "[heartbeat] --host が IPv4 アドレスとして解釈できない: %s\n", host.c_str());
        ::close(fd);
        return 2;
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::printf("[heartbeat] %s:%u へ %.1fHz で送信する。Ctrl-C で停止\n", host.c_str(), port, rate);
    std::printf("[heartbeat] ⚠️ 停止するとロボットの巡回も止まる(D-31)\n");

    // 起動ごとに振り直す乱数。受信側はこれで「送信プログラムが再起動した」ことを
    // 見分け、seq が 1 に戻っても古いパケットと誤認しない。
    std::random_device rd;
    std::mt19937_64 gen(rd());
    const std::uint64_t session_id = gen();
    std::printf("[heartbeat] session_id=%016lx\n", static_cast<unsigned long>(session_id));

    const auto period = std::chrono::duration<double>(1.0 / rate);
    std::uint64_t seq = 0;
    std::uint64_t sent = 0, failed = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!g_stop) {
        g1_sdk_bridge::HeartbeatPacket pkt;
        pkt.session_id = session_id;
        pkt.seq = ++seq;
        pkt.send_monotonic_ns = g1_sdk_bridge::MonotonicNs();
        std::array<std::uint8_t, g1_sdk_bridge::kHeartbeatWireSize> buf{};
        pkt.Encode(buf.data());

        const ssize_t n = ::sendto(fd, buf.data(), buf.size(), 0,
                                   reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
        if (n == static_cast<ssize_t>(buf.size())) {
            ++sent;
        } else {
            ++failed;
            // ⚠️ 送信失敗でここを終了させない。リンクが落ちている間も動き続け、
            // 復旧したら自動的に送信を再開する。終了してしまうと、復旧しても
            // 誰も送信を再開せず、ロボットが止まったままになる。
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_report > std::chrono::seconds(5)) {
            std::printf("[heartbeat] seq=%lu 送信=%lu 失敗=%lu\n",
                        static_cast<unsigned long>(seq), static_cast<unsigned long>(sent),
                        static_cast<unsigned long>(failed));
            last_report = now;
        }
        std::this_thread::sleep_for(period);
    }

    std::printf("[heartbeat] 終了する(送信=%lu 失敗=%lu)。**PC2 側は途絶と判定する**\n",
                static_cast<unsigned long>(sent), static_cast<unsigned long>(failed));
    ::close(fd);
    return 0;
}
