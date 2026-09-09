#include "g1_sdk_bridge/sdk_bridge_process.hpp"

#include <cmath>
#include <stdexcept>

namespace g1_sdk_bridge {

namespace {
std::array<double, 3> IntegratePose(const std::array<double, 3>& pose, const std::array<double, 3>& vel, double dt) {
    // デモ用の簡易積分。実機ではodometryはLIOから供給される(D-19)ため、この積分自体は本番では使わない。
    double x = pose[0], y = pose[1], yaw = pose[2];
    const double vx = vel[0], vy = vel[1], omega = vel[2];
    x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    yaw += omega * dt;
    return {x, y, yaw};
}

bool AnyNonZero(const std::array<double, 3>& v) { return v[0] != 0.0 || v[1] != 0.0 || v[2] != 0.0; }
}  // namespace

// --- MockMoveBackend ---------------------------------------------------

void MockMoveBackend::SetVelocity(double vx, double vy, double omega, double duration) {
    bool should_fail = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        calls_.push_back(Call{vx, vy, omega, duration, MonotonicNs()});
        if (fail_remaining_ > 0) {
            fail_remaining_ -= 1;
            should_fail = true;
        }
    }
    if (should_fail) {
        throw std::runtime_error("mock SDK call failure (test injected)");
    }
}

void MockMoveBackend::FailNext(int n) {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_remaining_ = n;
}

std::optional<MockMoveBackend::Call> MockMoveBackend::LastCall() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (calls_.empty()) return std::nullopt;
    return calls_.back();
}

std::vector<MockMoveBackend::Call> MockMoveBackend::AllCalls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
}

std::size_t MockMoveBackend::CallCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_.size();
}

// --- SdkBridgeConfig -----------------------------------------------------

void SdkBridgeConfig::Validate() const {
    const double send_period = 1.0 / cmd_rate_hz;
    if (!(send_period < sdk_command_duration_s && sdk_command_duration_s <= cmd_timeout_s)) {
        throw std::invalid_argument(
            "sdk_command_duration_s は 送信周期(1/cmd_rate_hz) より長く、"
            "cmd_timeout_s 以下でなければならない(D-27の原則)");
    }
}

// --- SdkBridgeProcess ------------------------------------------------------

SdkBridgeProcess::SdkBridgeProcess(SdkBridgeConfig config, MoveBackend& move_backend)
    : cfg_(std::move(config)),
      move_(move_backend),
      cmd_server_(cfg_.cmd_sock_path, sizeof(CmdWire)),
      state_server_(cfg_.state_sock_path, sizeof(StateWire)) {
    cfg_.Validate();
}

SdkBridgeProcess::~SdkBridgeProcess() { Stop(); }

void SdkBridgeProcess::Start() {
    // D-11: 起動直後に必ずゼロ速度を送信する。クラッシュ後の再起動でも前回速度を残さない
    move_.SetVelocity(0.0, 0.0, 0.0, cfg_.sdk_command_duration_s);

    running_ = true;
    accept_thread_ = std::thread(&SdkBridgeProcess::AcceptLoop, this);
    cmd_recv_thread_ = std::thread(&SdkBridgeProcess::CmdRecvLoop, this);
    periodic_thread_ = std::thread(&SdkBridgeProcess::PeriodicLoop, this);
}

void SdkBridgeProcess::Stop() {
    running_ = false;
    if (accept_thread_.joinable()) accept_thread_.join();
    if (cmd_recv_thread_.joinable()) cmd_recv_thread_.join();
    if (periodic_thread_.joinable()) periodic_thread_.join();

    std::lock_guard<std::mutex> lock(mutex_);
    if (cmd_endpoint_.has_value()) cmd_endpoint_->Close();
    if (state_endpoint_.has_value()) state_endpoint_->Close();
    cmd_endpoint_.reset();
    state_endpoint_.reset();
}

bool SdkBridgeProcess::ClearFault() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ != BridgeStatus::kFault) return false;
    status_ = BridgeStatus::kStandby;
    sdk_error_count_ = 0;
    return true;
}

BridgeStatus SdkBridgeProcess::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

int SdkBridgeProcess::sdk_error_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sdk_error_count_;
}

void SdkBridgeProcess::AcceptLoop() {
    while (running_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!cmd_endpoint_.has_value()) {
                auto ep = cmd_server_.AcceptIfPending();
                if (ep.has_value()) {
                    cmd_endpoint_ = std::move(ep);
                    last_cmd_.reset();  // 新規接続では新しい指令を待つ(古い指令を引き継がない)
                    if (status_ == BridgeStatus::kDisconnected) {
                        status_ = BridgeStatus::kStandby;
                    }
                }
            }
            if (!state_endpoint_.has_value()) {
                auto ep = state_server_.AcceptIfPending();
                if (ep.has_value()) {
                    state_endpoint_ = std::move(ep);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg_.poll_interval_s));
    }
}

void SdkBridgeProcess::CmdRecvLoop() {
    while (running_) {
        std::optional<std::vector<std::uint8_t>> raw;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (cmd_endpoint_.has_value()) {
                try {
                    raw = cmd_endpoint_->RecvLatest();
                } catch (const PeerClosed&) {
                    cmd_endpoint_->Close();
                    cmd_endpoint_.reset();
                    if (status_ != BridgeStatus::kFault) {
                        status_ = BridgeStatus::kDisconnected;
                    }
                }
            }
        }
        if (raw.has_value()) {
            try {
                const CmdPacket pkt = CmdPacket::Decode(raw->data(), raw->size());
                std::lock_guard<std::mutex> lock(mutex_);
                last_cmd_ = pkt;
            } catch (const ProtocolError&) {
                // 破損パケットは無視する
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg_.poll_interval_s));
    }
}

void SdkBridgeProcess::PeriodicLoop() {
    const double period = 1.0 / cfg_.cmd_rate_hz;
    auto next_tick = std::chrono::steady_clock::now();
    while (running_) {
        Tick(period);
        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(period));
        const auto now = std::chrono::steady_clock::now();
        if (next_tick > now) {
            std::this_thread::sleep_for(next_tick - now);
        } else {
            next_tick = now;  // 遅延が蓄積した場合は仕切り直す
        }
    }
}

void SdkBridgeProcess::Tick(double dt) {
    std::optional<CmdPacket> cmd;
    BridgeStatus status;
    bool cmd_connected;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd = last_cmd_;
        status = status_;
        cmd_connected = cmd_endpoint_.has_value();
    }

    std::array<double, 3> effective{0.0, 0.0, 0.0};
    BridgeStatus new_status;

    if (!cmd_connected) {
        new_status = BridgeStatus::kDisconnected;
    } else if (status == BridgeStatus::kFault) {
        // D-13: FAULT中は自動復帰せずゼロ速度を送り続ける。解除はClearFault()のみ
        new_status = BridgeStatus::kFault;
    } else if (!cmd.has_value() || cmd->AgeSeconds() > cfg_.cmd_timeout_s) {
        // SDK側watchdog(D-09): 指令が無い、または古すぎる場合はゼロ速度
        new_status = BridgeStatus::kReady;
    } else {
        effective = {cmd->vx, cmd->vy, cmd->omega};
        new_status = AnyNonZero(effective) ? BridgeStatus::kNavigating : BridgeStatus::kReady;
    }

    const int error_count = ApplyMove(effective);
    if (error_count >= cfg_.max_sdk_errors) {
        new_status = BridgeStatus::kFault;
        effective = {0.0, 0.0, 0.0};  // FAULT遷移した回のstateは安全側に倒す
    }

    std::array<double, 3> pose;
    int err;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = new_status;
        pose_ = IntegratePose(pose_, effective, dt);
        pose = pose_;
        err = sdk_error_count_;
    }

    PublishState(new_status, pose, effective, err);
}

int SdkBridgeProcess::ApplyMove(const std::array<double, 3>& effective) {
    try {
        move_.SetVelocity(effective[0], effective[1], effective[2], cfg_.sdk_command_duration_s);
    } catch (const std::exception&) {
        std::lock_guard<std::mutex> lock(mutex_);
        sdk_error_count_ += 1;
        return sdk_error_count_;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    sdk_error_count_ = 0;  // 「連続」失敗回数なので成功でリセット(仕様書8章)
    return sdk_error_count_;
}

void SdkBridgeProcess::PublishState(BridgeStatus status, const std::array<double, 3>& pose,
                                     const std::array<double, 3>& effective, int err) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_endpoint_.has_value()) {
        return;
    }
    state_seq_ += 1;
    StatePacket pkt;
    pkt.seq = state_seq_;
    pkt.timestamp_ns = MonotonicNs();
    pkt.status = status;
    pkt.x = pose[0];
    pkt.y = pose[1];
    pkt.yaw = pose[2];
    pkt.vx = effective[0];
    pkt.vy = effective[1];
    pkt.omega = effective[2];
    pkt.sdk_error_count = static_cast<std::uint32_t>(err);
    const StateWire wire = pkt.Encode();
    try {
        state_endpoint_->SendLatest(&wire, sizeof(wire));
    } catch (const PeerClosed&) {
        state_endpoint_->Close();
        state_endpoint_.reset();
    }
}

}  // namespace g1_sdk_bridge
