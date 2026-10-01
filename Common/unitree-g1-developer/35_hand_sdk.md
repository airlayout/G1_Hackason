# Internal wiring dexterity hand control interface

> 出典: https://support.unitree.com/home/en/G1_developer/hand_sdk

Hand SDK Interface
Overview

The Hand SDK is an external control interface provided by the ai_sport module for the G1 dexterous hand / gripper. By publishing messages to the DDS topic rt/hand_sdk, a user process can inject commands for the 4 hand motors into ai_sport. The injected commands are linearly blended with ai_sport's own default hand commands via a user-supplied weight before being sent to the motors.

How It Works
Command Blending

Hand SDK blends the user command with ai_sport's default command and sends the result to the motors:

Motor_real = weight * Hand_SDK + (1 - weight) * G1_Cmd

where:

Hand_SDK — the command published by the user on rt/hand_sdk. All five fields (kp, kd, q, dq, tau) participate in the blend.
G1_Cmd — the default hand command computed internally by ai_sport for the current FSM.
weight ∈ [0, 1] — supplied by the user on every message.
Sign Convention

💡 Convention: positive motor direction means close; negative direction means open.

Both the Dex2-5 five-finger 2-DOF hand and the Dex1-1 parallel gripper follow this convention. Keep this in mind when setting tau.

DDS Interface
Item	Value
Topic	rt/hand_sdk
Message type	unitree_go::msg::dds_::MotorCmds_
Number of motors	4
MotorCmds_ Field Layout

Fields of cmds[i] (i = 0 ~ 3):

Field	Type	Meaning
mode	uint8	Only cmds[0].mode is used: stores weight * 100 as an integer in 0 ~ 100
q	float	Target joint position
dq	float	Target joint velocity
kp	float	Position-loop stiffness
kd	float	Velocity-loop damping
tau	float	Feed-forward torque (positive = close, negative = open)

💡 Weight encoding: weight is stored as an integer (0 ~ 100) in cmds[0].mode; decode with weight = cmds[0].mode / 100.0. The mode field of the other three motors is unused.

C++ Example

The example below (hand_sdk_example) takes full control of the hand (weight = 1.0) and toggles tau between +0.3 and -0.3 every second, alternating between "close" and "open".

class HandSdk {
public:
    static constexpr int kMotorNum = 4;

    explicit HandSdk(const std::string& topic = "rt/hand_sdk")
        : publisher_(std::make_shared<unitree::robot::ChannelPublisher<
              unitree_go::msg::dds_::MotorCmds_>>(topic)) {
        publisher_->InitChannel();
        msg_.cmds().resize(kMotorNum);
    }

    void set_weight(float w) {
        msg_.cmds()[0].mode(
            static_cast<uint8_t>(std::clamp(w, 0.f, 1.f) * 100.f));
    }

    // Positive tau closes the hand, negative tau opens it.
    void set_tau(float tau) {
        for (int i = 0; i < kMotorNum; ++i) {
            msg_.cmds()[i].tau(tau);
        }
    }

    void write() { publisher_->Write(msg_); }

private:
    std::shared_ptr<unitree::robot::ChannelPublisher<
        unitree_go::msg::dds_::MotorCmds_>> publisher_;
    unitree_go::msg::dds_::MotorCmds_ msg_;
};

int main(int argc, char** argv) {
    unitree::robot::ChannelFactory::Instance()->Init(
        0, argc > 1 ? argv[1] : "");

    HandSdk hand_sdk;
    hand_sdk.set_weight(1.0f);

    float tau = 0.3f;
    while (true) {
        tau = -tau;
        hand_sdk.set_tau(tau);
        hand_sdk.write();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

Before running the example, verify that:

The ai_sport process is running on the robot;
The robot is not in the damping state;
A compatible dexterous hand / gripper is installed.
Notes

⚠️ Disabled in damping state: while the robot is in the damping state, user commands are not forwarded to the motors. After leaving the damping state, the user must re-set weight explicitly.

⚠️ Ramp weight smoothly: if ai_sport's default hand command differs significantly from the user's target, stepping weight from 0 to 1 may cause an abrupt motion. Ramp up gradually.

⚠️ Sign convention: positive tau closes the hand, negative tau opens it — unrelated to joint-angle sign conventions used elsewhere on the arm.

💡 Auto-fallback on timeout: if the user process stops publishing, Hand SDK falls back to ai_sport's default behaviour automatically — no explicit cancel call is required.