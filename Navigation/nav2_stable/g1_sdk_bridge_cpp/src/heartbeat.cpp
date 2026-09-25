#include "g1_sdk_bridge/heartbeat.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>

namespace g1_sdk_bridge {
namespace {

double DefaultNowSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void PutU32Le(std::uint8_t* p, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
}
void PutU64Le(std::uint8_t* p, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) p[i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
}
std::uint32_t GetU32Le(const std::uint8_t* p) {
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(p[i]) << (8 * i);
    return v;
}
std::uint64_t GetU64Le(const std::uint8_t* p) {
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
    return v;
}

}  // namespace

// --- HeartbeatPacket -------------------------------------------------------

void HeartbeatPacket::Encode(void* out) const {
    auto* p = static_cast<std::uint8_t*>(out);
    PutU32Le(p, kHeartbeatMagic);
    p[4] = kHeartbeatVersion;
    PutU64Le(p + 5, session_id);
    PutU64Le(p + 13, seq);
    PutU64Le(p + 21, send_monotonic_ns);
}

std::optional<HeartbeatPacket> HeartbeatPacket::Decode(const void* data, std::size_t size) {
    if (size != kHeartbeatWireSize) {
        return std::nullopt;
    }
    const auto* p = static_cast<const std::uint8_t*>(data);
    if (GetU32Le(p) != kHeartbeatMagic || p[4] != kHeartbeatVersion) {
        return std::nullopt;
    }
    HeartbeatPacket pkt;
    pkt.session_id = GetU64Le(p + 5);
    pkt.seq = GetU64Le(p + 13);
    pkt.send_monotonic_ns = GetU64Le(p + 21);
    return pkt;
}

// --- HeartbeatMonitor ------------------------------------------------------

HeartbeatMonitor::HeartbeatMonitor(double timeout_s, std::function<double()> now_fn)
    : timeout_s_(timeout_s), now_(now_fn ? std::move(now_fn) : std::function<double()>(DefaultNowSeconds)) {
    if (timeout_s_ <= 0.0) {
        throw std::invalid_argument("heartbeat の timeout_s は正でなければならない");
    }
}

bool HeartbeatMonitor::OnDatagram(const void* data, std::size_t size) {
    const auto pkt = HeartbeatPacket::Decode(data, size);
    if (!pkt.has_value()) {
        ++rejected_;
        return false;
    }
    // 送信側が再起動すると seq は 1 に戻る。session_id で区別しないと、
    // 再起動後のパケットを「古い」と見なして**永久に拒否し続ける**。
    const bool new_session = !ever_received_ || pkt->session_id != session_id_;
    // UDP は順序を保証しないので、同一セッション内では seq が進んだものだけを受理する。
    // 遅れて届いた古いパケットで「生きている」と誤認しないため。
    if (!new_session && pkt->seq <= last_seq_) {
        ++rejected_;
        return false;
    }
    session_id_ = pkt->session_id;
    last_seq_ = pkt->seq;
    last_rx_time_ = now_();
    ever_received_ = true;
    ++accepted_;
    return true;
}

bool HeartbeatMonitor::Alive() const {
    if (!ever_received_) {
        return false;  // 一度も受けていない状態を「生きている」と扱わない
    }
    return (now_() - last_rx_time_) <= timeout_s_;
}

std::optional<double> HeartbeatMonitor::SecondsSinceLast() const {
    if (!ever_received_) {
        return std::nullopt;
    }
    return now_() - last_rx_time_;
}

// --- HeartbeatReceiver -----------------------------------------------------

HeartbeatReceiver::HeartbeatReceiver(std::uint16_t port, double timeout_s, const std::string& bind_address)
    : port_(port), monitor_(timeout_s) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        throw std::runtime_error(std::string("heartbeat 用 UDP ソケットを作れない: ") + std::strerror(errno));
    }
    // ⚠️ **SO_REUSEADDR は付けない。**
    // Linux では UDP に SO_REUSEADDR を付けると**同じ addr:port に二重 bind できてしまう**。
    // cmd_router を誤って二重起動したとき、datagram が両者に振り分けられて
    // **どちらも「取りこぼしている」= 途絶したと誤検知する**(単体テストで発覚)。
    // TIME_WAIT の回避は TCP の話なので、受信専用の UDP に付ける利点は無い。
    // 二重 bind は EADDRINUSE で失敗させ、起動時に気づけるようにする。

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (bind_address.empty()) {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, bind_address.c_str(), &addr.sin_addr) != 1) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("heartbeat の bind アドレスが不正: " + bind_address);
    }
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        const std::string why = std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("heartbeat のポートを bind できない(" + std::to_string(port_) + "): " + why);
    }

    // Stop() で速やかに抜けられるよう受信にタイムアウトを付ける。
    // これが無いと recvfrom がブロックしたままスレッドを join できない。
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;  // 200ms
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

HeartbeatReceiver::~HeartbeatReceiver() {
    Stop();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void HeartbeatReceiver::Start() {
    if (running_) {
        return;
    }
    running_ = true;
    thread_ = std::thread(&HeartbeatReceiver::RecvLoop, this);
}

void HeartbeatReceiver::Stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void HeartbeatReceiver::RecvLoop() {
    std::array<std::uint8_t, 64> buf{};
    while (running_) {
        const ssize_t n = ::recvfrom(fd_, buf.data(), buf.size(), 0, nullptr, nullptr);
        if (n <= 0) {
            continue;  // タイムアウト(EAGAIN)。running_ を見直してループする
        }
        std::lock_guard<std::mutex> lock(mutex_);
        monitor_.OnDatagram(buf.data(), static_cast<std::size_t>(n));
    }
}

bool HeartbeatReceiver::Alive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return monitor_.Alive();
}
bool HeartbeatReceiver::EverReceived() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return monitor_.EverReceived();
}
std::optional<double> HeartbeatReceiver::SecondsSinceLast() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return monitor_.SecondsSinceLast();
}
std::uint64_t HeartbeatReceiver::accepted() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return monitor_.accepted();
}
std::uint64_t HeartbeatReceiver::rejected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return monitor_.rejected();
}

}  // namespace g1_sdk_bridge
