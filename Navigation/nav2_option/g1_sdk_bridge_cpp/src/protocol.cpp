#include "g1_sdk_bridge/protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

namespace g1_sdk_bridge {

std::uint64_t MonotonicNs() {
    using namespace std::chrono;
    return static_cast<std::uint64_t>(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}

namespace {
double AgeSecondsFrom(std::uint64_t timestamp_ns, std::optional<std::uint64_t> now_ns) {
    const std::uint64_t now = now_ns.has_value() ? *now_ns : MonotonicNs();
    // 単調時計なので理論上は起きないはずだが、負にならないようクランプする(Python版と同じ設計)。
    if (now <= timestamp_ns) {
        return 0.0;
    }
    return static_cast<double>(now - timestamp_ns) / 1e9;
}
}  // namespace

CmdWire CmdPacket::Encode() const {
    CmdWire w{};
    w.magic = kMagic;
    w.seq = seq;
    w.timestamp_ns = timestamp_ns;
    w.vx = vx;
    w.vy = vy;
    w.omega = omega;
    return w;
}

CmdPacket CmdPacket::Decode(const void* data, std::size_t size) {
    if (size != sizeof(CmdWire)) {
        throw ProtocolError("CmdPacket: 想定サイズと異なるバイト数を受信した");
    }
    CmdWire w{};
    std::memcpy(&w, data, sizeof(CmdWire));
    if (w.magic != kMagic) {
        throw ProtocolError("CmdPacket: magic不一致。異なるプロトコルのデータが混入している");
    }
    CmdPacket pkt;
    pkt.seq = w.seq;
    pkt.timestamp_ns = w.timestamp_ns;
    pkt.vx = w.vx;
    pkt.vy = w.vy;
    pkt.omega = w.omega;
    return pkt;
}

double CmdPacket::AgeSeconds(std::optional<std::uint64_t> now_ns) const { return AgeSecondsFrom(timestamp_ns, now_ns); }

CmdPacket MakeCmd(std::uint64_t seq, double vx, double vy, double omega) {
    CmdPacket pkt;
    pkt.seq = seq;
    pkt.timestamp_ns = MonotonicNs();
    pkt.vx = vx;
    pkt.vy = vy;
    pkt.omega = omega;
    return pkt;
}

StateWire StatePacket::Encode() const {
    StateWire w{};
    w.magic = kMagic;
    w.seq = seq;
    w.timestamp_ns = timestamp_ns;
    w.status = static_cast<std::uint8_t>(status);
    w.x = x;
    w.y = y;
    w.yaw = yaw;
    w.vx = vx;
    w.vy = vy;
    w.omega = omega;
    w.sdk_error_count = sdk_error_count;
    return w;
}

StatePacket StatePacket::Decode(const void* data, std::size_t size) {
    if (size != sizeof(StateWire)) {
        throw ProtocolError("StatePacket: 想定サイズと異なるバイト数を受信した");
    }
    StateWire w{};
    std::memcpy(&w, data, sizeof(StateWire));
    if (w.magic != kMagic) {
        throw ProtocolError("StatePacket: magic不一致");
    }
    StatePacket pkt;
    pkt.seq = w.seq;
    pkt.timestamp_ns = w.timestamp_ns;
    pkt.status = static_cast<BridgeStatus>(w.status);
    pkt.x = w.x;
    pkt.y = w.y;
    pkt.yaw = w.yaw;
    pkt.vx = w.vx;
    pkt.vy = w.vy;
    pkt.omega = w.omega;
    pkt.sdk_error_count = w.sdk_error_count;
    return pkt;
}

double StatePacket::AgeSeconds(std::optional<std::uint64_t> now_ns) const { return AgeSecondsFrom(timestamp_ns, now_ns); }

}  // namespace g1_sdk_bridge
