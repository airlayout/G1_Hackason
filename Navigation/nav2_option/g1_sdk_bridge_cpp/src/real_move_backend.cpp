#include "g1_sdk_bridge/real_move_backend.hpp"

// unitree_sdk2 のヘッダを include するのは**このファイルだけ**。
// ヘッダ側に漏らさない理由は real_move_backend.hpp の先頭コメント参照。
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>

namespace g1_sdk_bridge {

struct RealMoveBackend::Impl {
    RealMoveBackendConfig config;
    // ⚠️ **LocoClient は ChannelFactory::Init() の後に構築しなければならない。**
    // メンバ実体として持つと Impl の構築時（= ChannelFactory::Init より前）に
    // LocoClient のコンストラクタが走り、**segfault する**（2026-09-09 に実機で踏んだ）。
    // 公式サンプル(g1_loco_client_example.cpp)も Init → client 構築の順になっている。
    // そのため unique_ptr で遅延構築する。
    std::unique_ptr<unitree::robot::g1::LocoClient> client;
    // LocoClient::SetVelocity は周期スレッドから呼ばれる。SDK側のスレッド安全性が
    // 保証されていないので、こちらで直列化しておく。
    std::mutex call_mutex;
    std::atomic<std::size_t> sent{0};
    std::atomic<std::size_t> blocked{0};
};

RealMoveBackend::RealMoveBackend(const RealMoveBackendConfig& config) : impl_(std::make_unique<Impl>()) {
    impl_->config = config;

    // ChannelFactory はプロセスに1つ。二重初期化すると SDK 側で例外になるため、
    // このプロセスでは本クラスの生成を1回だけにする前提で呼ぶ。
    unitree::robot::ChannelFactory::Instance()->Init(config.domain_id, config.network_interface);

    // ChannelFactory の初期化が済んだ**後**に構築する(上の Impl のコメント参照)
    impl_->client = std::make_unique<unitree::robot::g1::LocoClient>();
    impl_->client->Init();
    impl_->client->SetTimeout(static_cast<float>(config.rpc_timeout_s));

    std::printf("[real_backend] LocoClient を初期化した (iface=%s domain=%d timeout=%.1fs) / 発進ゲート=%s\n",
                config.network_interface.c_str(), config.domain_id, config.rpc_timeout_s,
                config.armed ? "開(armed)" : "閉(SDKを呼ばない)");
}

RealMoveBackend::~RealMoveBackend() {
    // 終了時にゼロ速度を送る。StopMove() は内部で duration の既定値(1.0秒)を使うため
    // D-27 の観点で使わず、設定した duration を明示して SetVelocity を呼ぶ。
    // デストラクタから例外を出さないよう必ず捕まえる。
    if (impl_ && impl_->client && impl_->config.armed) {
        try {
            std::lock_guard<std::mutex> lock(impl_->call_mutex);
            impl_->client->SetVelocity(0.f, 0.f, 0.f, static_cast<float>(impl_->config.max_duration_s));
            std::printf("[real_backend] 終了時にゼロ速度を送った\n");
        } catch (...) {
            std::fprintf(stderr, "[real_backend] 終了時のゼロ速度送信に失敗した\n");
        }
    }
}

void RealMoveBackend::SetVelocity(double vx, double vy, double omega, double duration) {
    // D-27: duration は常に有限で、0 より大きく上限以下でなければならない。
    // 上位(SafetyManager / SdkBridgeConfig)でも検証しているが、SDK を叩く直前でも確かめる。
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) || !std::isfinite(duration)) {
        throw std::runtime_error("SetVelocity に非有限値が渡された(D-27違反)");
    }
    if (duration <= 0.0 || duration > impl_->config.max_duration_s) {
        throw std::runtime_error("duration が許容範囲外(D-27違反): " + std::to_string(duration) +
                                 "s (上限 " + std::to_string(impl_->config.max_duration_s) + "s)");
    }

    if (!impl_->config.armed) {
        impl_->blocked.fetch_add(1);
        return;  // 発進ゲートが閉じている。SDK を一切呼ばない
    }

    std::lock_guard<std::mutex> lock(impl_->call_mutex);
    const int32_t ret = impl_->client->SetVelocity(static_cast<float>(vx), static_cast<float>(vy),
                                                   static_cast<float>(omega), static_cast<float>(duration));
    if (ret != 0) {
        throw std::runtime_error("LocoClient::SetVelocity が失敗した (ret=" + std::to_string(ret) + ")");
    }
    impl_->sent.fetch_add(1);
}

bool RealMoveBackend::armed() const { return impl_->config.armed; }
std::size_t RealMoveBackend::sent_count() const { return impl_->sent.load(); }
std::size_t RealMoveBackend::blocked_count() const { return impl_->blocked.load(); }

}  // namespace g1_sdk_bridge
