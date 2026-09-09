// Unix domain socket (SOCK_SEQPACKET) による cmd/state 伝送。
//
// Python版プロトタイプ(../../g1_sdk_bridge/ipc_transport.py)の1対1移植。
//
// Planning.md D-05 の決定:
// - SOCK_SEQPACKET を使う(メッセージ境界が保たれ、相手の接続断を検知できる)
// - 送信側は非ブロッキング。バッファ満杯時は古いものを捨て、最新値を優先する
// - cmdとstateは別ソケット(stateの高頻度配信がcmdのレイテンシに影響しないように)
//
// 役割: SDK側プロセスが両ソケットのサーバ(bind + listen)になる。
// systemdで常駐管理される側が固定のアドレスを持つ方が構成が単純なため。
// ROS側プロセス(safety manager)がクライアントとして接続する。

#pragma once

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_sdk_bridge {

// 相手がソケットを閉じた(プロセス終了・クラッシュ)ことを示す。
class PeerClosed : public std::runtime_error {
public:
    explicit PeerClosed(const std::string& what) : std::runtime_error(what) {}
};

// 接続済み SOCK_SEQPACKET ソケットの薄いラッパ。move-only(fdの二重closeを防ぐ)。
class SeqPacketEndpoint {
public:
    SeqPacketEndpoint(int fd, std::size_t max_payload);
    ~SeqPacketEndpoint();

    SeqPacketEndpoint(const SeqPacketEndpoint&) = delete;
    SeqPacketEndpoint& operator=(const SeqPacketEndpoint&) = delete;
    SeqPacketEndpoint(SeqPacketEndpoint&& other) noexcept;
    SeqPacketEndpoint& operator=(SeqPacketEndpoint&& other) noexcept;

    // 非ブロッキング送信。バッファ満杯(EAGAIN)なら黙って破棄しfalseを返す。
    //
    // 最新値優先の設計: 送れなかった1件は次にもっと新しい値が来て上書きされるだけなので、
    // 古い値を無理に送るより破棄する方が正しい。
    bool SendLatest(const void* data, std::size_t size);

    // 受信バッファに溜まっている分をすべて読み切り、最後の1件だけを返す。
    //
    // 古い未処理メッセージが残っていても、常に「今の値」を使うことで
    // 処理遅延によるコマンド滞留を防ぐ(latest-value semantics)。
    std::optional<std::vector<std::uint8_t>> RecvLatest();

    void Close();
    int fd() const { return fd_; }

private:
    int fd_;
    std::size_t max_payload_;
};

// bind + listen する側。SDK側プロセスが cmd/state それぞれで1つずつ持つ。
class SeqPacketServer {
public:
    SeqPacketServer(const std::string& path, std::size_t max_payload);
    ~SeqPacketServer();

    SeqPacketServer(const SeqPacketServer&) = delete;
    SeqPacketServer& operator=(const SeqPacketServer&) = delete;

    // 非ブロッキングでaccept。接続要求が無ければstd::nullopt。
    std::optional<SeqPacketEndpoint> AcceptIfPending();

    SeqPacketEndpoint AcceptBlocking(std::optional<double> timeout_seconds = std::nullopt);

    void Close();

private:
    std::string path_;
    std::size_t max_payload_;
    int listen_fd_;
};

// ROS側プロセスがSDK側プロセスへ接続する側。
SeqPacketEndpoint ConnectClient(const std::string& path, std::size_t max_payload, double timeout_seconds = 5.0);

}  // namespace g1_sdk_bridge
