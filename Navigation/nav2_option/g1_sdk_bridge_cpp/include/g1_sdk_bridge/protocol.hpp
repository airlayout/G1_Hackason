// ROS側プロセスとSDK2側プロセスの間で交換するIPCペイロードの定義。
//
// Python版プロトタイプ(../../g1_sdk_bridge/protocol.py)の1対1移植。
// ワイヤフォーマットは意図的にPython版と同一バイト配置にしてある(#pragma packで
// パディング無しにし、フィールド順序・型サイズをPython版のstruct書式("<IQQddd"等)に
// 合わせた)。ローカルUnix domain socket上でのみ使う前提のため、エンディアン変換は
// 行わない(同一ホスト・同一アーキテクチャ間の通信に限定される設計、D-05)。
//
// Planning.md D-05 の決定に基づく:
// - 固定長構造体 + magic + シーケンス番号 + CLOCK_MONOTONIC送信タイムスタンプ
// - cmdとstateでペイロードを分離する

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace g1_sdk_bridge {

constexpr std::uint32_t kMagic = 0x47314E56;  // "G1NV" をASCIIコードとして4バイトに詰めた値

enum class BridgeStatus : std::uint8_t {
    kDisconnected = 0,
    kStandby = 1,
    kReady = 2,
    kNavigating = 3,
    kFault = 4,
};

// std::chrono::steady_clock(CLOCK_MONOTONIC相当)基準のナノ秒タイムスタンプ。
std::uint64_t MonotonicNs();

class ProtocolError : public std::runtime_error {
public:
    explicit ProtocolError(const std::string& what) : std::runtime_error(what) {}
};

// --- Cmd: ROS側 -> SDK側 ---------------------------------------------------

#pragma pack(push, 1)
struct CmdWire {
    std::uint32_t magic;
    std::uint64_t seq;
    std::uint64_t timestamp_ns;
    double vx;
    double vy;
    double omega;
};
#pragma pack(pop)
static_assert(sizeof(CmdWire) == 44, "CmdWireのレイアウトがPython版protocol.pyのCmdPacketと一致しない");

// 速度指令。ROS側のSafety Managerが最終決定した安全な速度指令(/cmd_vel_safe相当)。
struct CmdPacket {
    std::uint64_t seq = 0;
    std::uint64_t timestamp_ns = 0;
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;

    CmdWire Encode() const;
    static CmdPacket Decode(const void* data, std::size_t size);

    // このパケットが作られてからの経過時間(秒)。stale判定に使う。
    double AgeSeconds(std::optional<std::uint64_t> now_ns = std::nullopt) const;

    bool operator==(const CmdPacket& other) const {
        return seq == other.seq && timestamp_ns == other.timestamp_ns && vx == other.vx && vy == other.vy &&
               omega == other.omega;
    }
};

CmdPacket MakeCmd(std::uint64_t seq, double vx, double vy, double omega);

// --- State: SDK側 -> ROS側 --------------------------------------------------

#pragma pack(push, 1)
struct StateWire {
    std::uint32_t magic;
    std::uint64_t seq;
    std::uint64_t timestamp_ns;
    std::uint8_t status;
    std::uint8_t _pad[3];
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
    double omega;
    std::uint32_t sdk_error_count;
};
#pragma pack(pop)
static_assert(sizeof(StateWire) == 76, "StateWireのレイアウトがPython版protocol.pyのStatePacketと一致しない");

// G1の状態。odometry相当(x,y,yaw,vx,vy,omega)とbridgeの健全性情報。
//
// x, y, yaw は G1起動時点を原点とする odom フレーム相当(累積ドリフトを持ちうる)。
// 実機ではSDK側プロセスがG1 stateから取得した値をそのまま詰める。
struct StatePacket {
    std::uint64_t seq = 0;
    std::uint64_t timestamp_ns = 0;
    BridgeStatus status = BridgeStatus::kDisconnected;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    std::uint32_t sdk_error_count = 0;

    StateWire Encode() const;
    static StatePacket Decode(const void* data, std::size_t size);

    double AgeSeconds(std::optional<std::uint64_t> now_ns = std::nullopt) const;

    bool operator==(const StatePacket& other) const {
        return seq == other.seq && timestamp_ns == other.timestamp_ns && status == other.status && x == other.x &&
               y == other.y && yaw == other.yaw && vx == other.vx && vy == other.vy && omega == other.omega &&
               sdk_error_count == other.sdk_error_count;
    }
};

}  // namespace g1_sdk_bridge
