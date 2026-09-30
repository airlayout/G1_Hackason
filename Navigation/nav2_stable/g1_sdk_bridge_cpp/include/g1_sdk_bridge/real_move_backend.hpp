// 実機のG1へ速度指令を送る MoveBackend。unitree_sdk2(C++)の LocoClient を使う。
//
// ⚠️ **このヘッダは unitree_sdk2 のヘッダを一切 include しない**(pimplで隠している)。
// ROS側(g1_ws)は本ディレクトリのソースを相対パスで直接コンパイルするため、ここにSDKの
// ヘッダが漏れるとROS側のビルドがSDKに依存してしまい、D-08(SDK側プロセスをcolcon
// workspace外の独立プロジェクトに保つ)の前提が崩れる。
//
// ## D-27 の遵守
//
// `Move()` も `SwitchMoveMode(true)` も**使わない**。`SetVelocity(vx,vy,omega,duration)` に
// 常に有限の duration を明示して渡す。`Move(vx,vy,vyaw)` は内部で duration=1.0秒
// (`continous_move=true` なら 864000秒≒10日)を渡すため、Bridge が死んだときに
// G1 が歩き続ける危険がある。実装側でも duration の範囲を検証して弾く。
//
// ## 発進ゲート(armed)
//
// `armed=false` の間は **SDK を一切呼ばない**(呼び出し回数だけ数える)。
// `Navigation/real/loco_driver.py` の `--arm` と同じ考え方で、ROS側の状態機械
// (`SafetyManager`)とは独立した防御層になる。配線ミスや誤起動でプロセスを立ち上げただけでは
// 機体が動かないことを構造的に保証する(D-10 の「防御を二重化する」方針と同じ)。

#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "g1_sdk_bridge/sdk_bridge_process.hpp"

namespace g1_sdk_bridge {

struct RealMoveBackendConfig {
    // ChannelFactory に渡すネットワークIF。G1内蔵スイッチ側のIF名(PC2では eth0)
    std::string network_interface = "eth0";
    int domain_id = 0;
    double rpc_timeout_s = 10.0;

    // 発進ゲート。false の間は SDK を呼ばない
    bool armed = false;

    // duration の受け付け上限[s]。D-27 の原則(送信周期 < duration <= cmd_timeout)に対し、
    // 明らかに外れた値を実装側でも弾くための保険。既定は cmd_timeout の想定最大値。
    double max_duration_s = 1.0;
};

class RealMoveBackend : public MoveBackend {
public:
    explicit RealMoveBackend(const RealMoveBackendConfig& config);
    ~RealMoveBackend() override;

    RealMoveBackend(const RealMoveBackend&) = delete;
    RealMoveBackend& operator=(const RealMoveBackend&) = delete;

    // SDK呼び出しが失敗したら std::runtime_error を投げる(MoveBackend の規約)。
    // armed=false のときは何も送らず、正常終了する。
    void SetVelocity(double vx, double vy, double omega, double duration) override;

    bool armed() const;
    // SDK へ実際に送った回数
    std::size_t sent_count() const;
    // 発進ゲートで止めた回数
    std::size_t blocked_count() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace g1_sdk_bridge
