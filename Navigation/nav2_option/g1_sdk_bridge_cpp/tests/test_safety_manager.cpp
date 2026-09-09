#include "g1_sdk_bridge/safety_manager.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace g1_sdk_bridge;

namespace {

// モノトニック時計のテスト用スタブ。明示的にしか進まない。
class FakeClock {
public:
    double now() const { return t_; }
    void Advance(double dt) { t_ += dt; }
    NowFn AsNowFn() {
        return [this]() { return this->now(); };
    }

private:
    double t_ = 0.0;
};

}  // namespace

// --- 純粋関数(clamp/deadband/accel_limit) -----------------------------

TEST(PureFilters, ClampLimitsEachAxis) {
    SafetyLimits limits;
    limits.max_vx = 0.2;
    limits.max_vy = 0.1;
    limits.max_wz = 0.3;
    Vel out = Clamp({1.0, 1.0, 1.0}, limits);
    EXPECT_DOUBLE_EQ(out[0], 0.2);
    EXPECT_DOUBLE_EQ(out[1], 0.1);
    EXPECT_DOUBLE_EQ(out[2], 0.3);

    out = Clamp({-1.0, -1.0, -1.0}, limits);
    EXPECT_DOUBLE_EQ(out[0], -0.2);
    EXPECT_DOUBLE_EQ(out[1], -0.1);
    EXPECT_DOUBLE_EQ(out[2], -0.3);

    out = Clamp({0.05, 0.0, 0.1}, limits);
    EXPECT_DOUBLE_EQ(out[0], 0.05);
    EXPECT_DOUBLE_EQ(out[1], 0.0);
    EXPECT_DOUBLE_EQ(out[2], 0.1);
}

TEST(PureFilters, DeadbandZeroesSmallValues) {
    SafetyLimits limits;
    limits.min_vx = 0.03;
    limits.min_wz = 0.03;
    const Vel out = ApplyDeadband({0.02, 0.05, 0.01}, limits);
    EXPECT_DOUBLE_EQ(out[0], 0.0);
    EXPECT_DOUBLE_EQ(out[2], 0.0);
    EXPECT_DOUBLE_EQ(out[1], 0.05);  // vyはD-14の対象外(MVPではmax_vy=0で別途ゼロになる)
}

TEST(PureFilters, DeadbandPassesThroughValuesAboveThreshold) {
    SafetyLimits limits;
    limits.min_vx = 0.03;
    limits.min_wz = 0.03;
    const Vel out = ApplyDeadband({0.05, 0.0, 0.10}, limits);
    EXPECT_DOUBLE_EQ(out[0], 0.05);
    EXPECT_DOUBLE_EQ(out[2], 0.10);
}

TEST(PureFilters, AccelLimitCapsRateOfChange) {
    SafetyLimits limits;
    limits.max_ax = 0.2;
    limits.max_ay = 0.15;
    limits.max_awz = 0.4;
    // 0 -> 1.0 へ一気に上げようとしても、dt=0.1sなら max_ax*dt=0.02までしか動けない
    const Vel out = AccelLimit({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, 0.1, limits);
    EXPECT_NEAR(out[0], 0.02, 1e-9);
    EXPECT_NEAR(out[1], 0.015, 1e-9);
    EXPECT_NEAR(out[2], 0.04, 1e-9);
}

TEST(PureFilters, AccelLimitDoesNotOvershootSmallTargets) {
    SafetyLimits limits;
    limits.max_ax = 0.2;
    const Vel out = AccelLimit({0.0, 0.0, 0.0}, {0.01, 0.0, 0.0}, 0.1, limits);
    EXPECT_NEAR(out[0], 0.01, 1e-9);  // 上限より小さい変化はそのまま通る
}

// --- SafetyManagerの状態機械 --------------------------------------------

class SafetyManagerStateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        limits_.cmd_timeout_s = 0.30;
        mgr_ = std::make_unique<SafetyManager>(
            limits_, [this](double vx, double vy, double wz) { sent_.push_back({vx, vy, wz}); }, clock_.AsNowFn());
    }

    void GotoNavigating() {
        mgr_->OnBridgeConnected();
        mgr_->MarkReady();
        ASSERT_TRUE(mgr_->EnableNavigation(true));
        ASSERT_EQ(mgr_->state(), NavState::kNavigating);
    }

    FakeClock clock_;
    SafetyLimits limits_;
    std::vector<Vel> sent_;
    std::unique_ptr<SafetyManager> mgr_;
};

TEST_F(SafetyManagerStateMachineTest, InitialStateIsDisconnected) {
    EXPECT_EQ(mgr_->state(), NavState::kDisconnected);
}

TEST_F(SafetyManagerStateMachineTest, FullHappyPathTransitions) {
    mgr_->OnBridgeConnected();
    EXPECT_EQ(mgr_->state(), NavState::kStandby);
    mgr_->MarkReady();
    EXPECT_EQ(mgr_->state(), NavState::kReady);
    EXPECT_TRUE(mgr_->EnableNavigation(true));
    EXPECT_EQ(mgr_->state(), NavState::kNavigating);
    EXPECT_TRUE(mgr_->EnableNavigation(false));
    EXPECT_EQ(mgr_->state(), NavState::kReady);
}

TEST_F(SafetyManagerStateMachineTest, EnableNavigationFailsOutsideReady) {
    // DISCONNECTED状態でいきなりNAVIGATINGにはできない(仕様書7章の遷移制約)
    EXPECT_FALSE(mgr_->EnableNavigation(true));
    EXPECT_EQ(mgr_->state(), NavState::kDisconnected);
}

TEST_F(SafetyManagerStateMachineTest, NonzeroVelocityOnlySentWhileNavigating) {
    // READY状態でNav2からTwistが来ても、非ゼロ速度は送らない(仕様書8章の状態別許可表)
    mgr_->OnBridgeConnected();
    mgr_->MarkReady();
    const Vel out = mgr_->OnNavTwist(0.5, 0.0, 0.0);
    EXPECT_EQ(out, (Vel{0.0, 0.0, 0.0}));
    EXPECT_EQ(sent_.back(), (Vel{0.0, 0.0, 0.0}));
}

TEST_F(SafetyManagerStateMachineTest, NavTwistIsClampedAndForwardedWhileNavigating) {
    GotoNavigating();
    SafetyLimits limits;  // 既定値(2026-09-09 実測: max_vx=0.30, max_ax=0.20 は未測定の仮値)
    mgr_->set_limits(limits);
    // AccelLimit が頭打ちにならないだけの時間を進めてから初回指令を送る。
    // 必要な dt は max_vx / max_ax = 0.30 / 0.20 = 1.5s なので、余裕を見て 2.0s 進める。
    // ⚠️ 以前は「max_ax*dt == max_vx」という偶然の一致に依存していたため、
    // max_vx を 0.20 から 0.30 に変えた時点でこのテストが落ちた。値に依存しない形に直した。
    clock_.Advance(limits.max_vx / limits.max_ax + 0.5);
    const Vel out = mgr_->OnNavTwist(1.0, 0.0, 0.0);  // 上限超えの指令
    EXPECT_EQ(out, (Vel{limits.max_vx, 0.0, 0.0}));
    EXPECT_EQ(sent_.back(), out);
}

TEST_F(SafetyManagerStateMachineTest, NavTwistRampsUpGraduallyRightAfterEnable) {
    // enable直後(dt≈0)に大きな指令が来ても、AccelLimitでいきなり全開にはならないことを確認する。
    GotoNavigating();
    clock_.Advance(0.05);  // 20Hz相当の1周期分だけ進める
    const Vel out = mgr_->OnNavTwist(1.0, 0.0, 0.0);
    const double expected_vx = limits_.max_ax * 0.05;  // 0.2 * 0.05 = 0.01
    EXPECT_NEAR(out[0], expected_vx, 1e-9);
    EXPECT_LT(out[0], limits_.max_vx);  // まだ上限には到達していない

    clock_.Advance(0.05);
    const Vel out2 = mgr_->OnNavTwist(1.0, 0.0, 0.0);
    EXPECT_GT(out2[0], out[0]);  // 徐々に増えていく
}

TEST_F(SafetyManagerStateMachineTest, EStopOverridesNavigatingAndSendsZero) {
    GotoNavigating();
    mgr_->EStop();
    EXPECT_EQ(mgr_->state(), NavState::kEStop);
    EXPECT_EQ(sent_.back(), (Vel{0.0, 0.0, 0.0}));
}

TEST_F(SafetyManagerStateMachineTest, FaultDoesNotOverrideEStop) {
    GotoNavigating();
    mgr_->EStop();
    mgr_->OnTfStale();  // E_STOP中にFAULT条件が来ても上書きしない
    EXPECT_EQ(mgr_->state(), NavState::kEStop);
}

TEST_F(SafetyManagerStateMachineTest, ClearEStopRequiresEStopState) {
    EXPECT_FALSE(mgr_->ClearEStop());  // DISCONNECTEDからは解除できない
    GotoNavigating();
    mgr_->EStop();
    EXPECT_TRUE(mgr_->ClearEStop());
    EXPECT_EQ(mgr_->state(), NavState::kStandby);
}

TEST_F(SafetyManagerStateMachineTest, ClearFaultRequiresFaultState) {
    EXPECT_FALSE(mgr_->ClearFault());
    GotoNavigating();
    mgr_->OnTfStale();
    EXPECT_EQ(mgr_->state(), NavState::kFault);
    EXPECT_TRUE(mgr_->ClearFault());
    EXPECT_EQ(mgr_->state(), NavState::kStandby);
}

TEST_F(SafetyManagerStateMachineTest, TickWatchdogFaultsOnCmdTimeout) {
    // D-10: ROS側watchdog。指令が来なくなってからcmd_timeoutを超えたら明示的にゼロを送りFAULTへ
    GotoNavigating();
    mgr_->OnNavTwist(0.1, 0.0, 0.0);
    clock_.Advance(limits_.cmd_timeout_s + 0.01);
    mgr_->Tick();
    EXPECT_EQ(mgr_->state(), NavState::kFault);
    EXPECT_EQ(sent_.back(), (Vel{0.0, 0.0, 0.0}));
}

TEST_F(SafetyManagerStateMachineTest, TickDoesNotFaultBeforeFirstTwistEvenAfterTimeoutElapsed) {
    // 2026-09-09にNav2統合dry-runで発見: Nav2はGoal計画に数百ms〜数秒かかることがあり、
    // enable直後にcmd_timeoutのカウントを始めると最初の指令が届く前にFAULTへ誤って遷移する。
    // 最初の指令を受け取るまではTick()のタイムアウト判定を待機させる。
    GotoNavigating();
    clock_.Advance(limits_.cmd_timeout_s * 10);  // cmd_timeoutを大幅に超えて経過させる
    mgr_->Tick();
    EXPECT_EQ(mgr_->state(), NavState::kNavigating);  // まだFAULTにならない

    // 最初の指令が来た後は、通常どおりcmd_timeoutが効く
    mgr_->OnNavTwist(0.1, 0.0, 0.0);
    clock_.Advance(limits_.cmd_timeout_s + 0.01);
    mgr_->Tick();
    EXPECT_EQ(mgr_->state(), NavState::kFault);
}

TEST_F(SafetyManagerStateMachineTest, TickDoesNotFaultWithinTimeout) {
    GotoNavigating();
    mgr_->OnNavTwist(0.1, 0.0, 0.0);
    clock_.Advance(limits_.cmd_timeout_s - 0.05);
    mgr_->Tick();
    EXPECT_EQ(mgr_->state(), NavState::kNavigating);
}

TEST_F(SafetyManagerStateMachineTest, BridgeDisconnectForcesDisconnectedAndZero) {
    GotoNavigating();
    mgr_->OnBridgeDisconnected();
    EXPECT_EQ(mgr_->state(), NavState::kDisconnected);
    EXPECT_EQ(sent_.back(), (Vel{0.0, 0.0, 0.0}));
}
