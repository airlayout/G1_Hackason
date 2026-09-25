// 操作PC の生存監視(heartbeat)。Planning.md **D-31** / Phase 2c 作業項目10。
// 設計の根拠は findings/operator_heartbeat_design.md。
//
// ## 何を解決するのか
//
// 2026-09-09 の実測で、走行中に操作PC側のリンクを切っても **ロボットは 4.045 秒
// 歩き続け、約 0.85m 前進した**。原因は `sshd` が TCP 切断を 14 秒検知せず
// SIGHUP を出さなかったこと。既存の 3 層(duration 満了 / SDK側watchdog /
// ROS側watchdog)は **いずれも「オンボード側の誰かが死ぬこと」を検知する仕組み**で、
// 通信断ではオンボード側は全員無事なので誰も気づかない。
//
// ## なぜ UDP なのか
//
// ⚠️ **TCP の切断検知に依存してはいけない。今回の事故の原因がまさにそれ。**
// UDP なら「届かない」がそのまま「届かない」として現れる。再送もハンドシェイクも
// 無いので、リンクが死ねば受信が即座に止まる。
//
// ## 時刻について
//
// パケットは送信側の単調時刻を載せるが、**生存判定には使わない**。
// PC 間の時刻同期を前提にしないため。判定は受信側のローカル時計で測った
// 「最後に受信してからの経過時間」だけで行う。送信時刻はログと遅延の可視化用。

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace g1_sdk_bridge {

// "G1HB" をASCIIコードとして4バイトに詰めた値
constexpr std::uint32_t kHeartbeatMagic = 0x47314842;
constexpr std::uint8_t kHeartbeatVersion = 1;
constexpr std::size_t kHeartbeatWireSize = 29;  // magic(4) + version(1) + session(8) + seq(8) + ns(8)
constexpr std::uint16_t kDefaultHeartbeatPort = 47311;

// 操作PC → PC2 に流れる heartbeat パケット。
//
// ⚠️ **エンコードは明示的なリトルエンディアンで行う。**
// IPC 用の CmdWire/StateWire は「同一ホスト・同一アーキテクチャ」を前提に
// 構造体をそのまま送っているが、heartbeat は**別マシン間**を渡る。
// 操作PC(x86_64) と PC2(aarch64) はどちらもリトルエンディアンなので実害は
// 出ない見込みだが、前提を暗黙にしておく理由が無いので明示的に詰める。
struct HeartbeatPacket {
    // 送信プロセスの起動ごとに振り直す乱数。**seq はこの中でのみ単調**。
    // ⚠️ これが無いと、送信プログラムを再起動したとき seq が 1 に戻り、
    // 受信側が「古いパケット」として**永久に拒否し続ける**(実装中に踏んだ)。
    // 運用で送信を再起動したらロボットが二度と動けなくなる。
    std::uint64_t session_id = 0;
    std::uint64_t seq = 0;
    std::uint64_t send_monotonic_ns = 0;  // 送信側の時計。**判定には使わない**(上記参照)

    // kHeartbeatWireSize バイトのバッファへ書き出す。
    void Encode(void* out) const;
    // 受け取ったバイト列を解釈する。magic/version/長さが違えば nullopt。
    static std::optional<HeartbeatPacket> Decode(const void* data, std::size_t size);
};

// 受信状況から「操作PC が生きているか」を判定する。
//
// ソケットを持たない純粋ロジックなので、単体テストで時計を差し替えて検証できる。
class HeartbeatMonitor {
public:
    // now_fn は秒単位の単調増加時刻を返す。省略時は steady_clock。
    explicit HeartbeatMonitor(double timeout_s, std::function<double()> now_fn = nullptr);

    // 受信したバイト列を渡す。受理したら true。
    // ⚠️ **同一セッション内で seq が巻き戻るパケットは捨てる。** UDP は順序を
    // 保証しないので、遅れて届いた古いパケットで「生きている」と誤認しないため。
    // session_id が変われば送信側が再起動したと見なし、seq を追い直す。
    bool OnDatagram(const void* data, std::size_t size);

    // 一度でも受信したか。**一度も受けていない状態を「生きている」と扱わないこと。**
    bool EverReceived() const { return ever_received_; }

    // 生存しているか。一度も受信していなければ false。
    bool Alive() const;

    // 最後に受信してからの経過秒。一度も受信していなければ nullopt。
    std::optional<double> SecondsSinceLast() const;

    std::uint64_t accepted() const { return accepted_; }
    std::uint64_t rejected() const { return rejected_; }
    std::uint64_t last_seq() const { return last_seq_; }
    std::uint64_t session_id() const { return session_id_; }
    double timeout_s() const { return timeout_s_; }

private:
    double timeout_s_;
    std::function<double()> now_;
    bool ever_received_ = false;
    std::uint64_t session_id_ = 0;
    std::uint64_t last_seq_ = 0;
    double last_rx_time_ = 0.0;
    std::uint64_t accepted_ = 0;
    std::uint64_t rejected_ = 0;
};

// UDP ソケットを開いて受信し、HeartbeatMonitor を更新し続ける。
//
// ⚠️ **バインドは既定で 0.0.0.0**(操作PC がどのアドレスから来ても受ける)。
// 受信専用でコマンドは一切受け付けないので、ここから機体を動かすことはできない。
class HeartbeatReceiver {
public:
    // bind_address が空なら 0.0.0.0。
    HeartbeatReceiver(std::uint16_t port, double timeout_s, const std::string& bind_address = "");
    ~HeartbeatReceiver();

    HeartbeatReceiver(const HeartbeatReceiver&) = delete;
    HeartbeatReceiver& operator=(const HeartbeatReceiver&) = delete;

    void Start();
    void Stop();

    bool Alive() const;
    bool EverReceived() const;
    std::optional<double> SecondsSinceLast() const;
    std::uint64_t accepted() const;
    std::uint64_t rejected() const;

private:
    void RecvLoop();

    std::uint16_t port_;
    int fd_ = -1;
    mutable std::mutex mutex_;
    HeartbeatMonitor monitor_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace g1_sdk_bridge
