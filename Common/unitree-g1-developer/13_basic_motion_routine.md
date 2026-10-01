# Basic Motion Routine

> 出典: https://support.unitree.com/home/en/G1_developer/basic_motion_routine

The low-level motion development routine implements resetting the robot from any initial joint position to the zero position, then swinging the G1 robot's ankle joints in two different modes, and printing Euler angle data at a certain frequency. The source code of the routine is located in unitree_sdk2/example/g1/low_level/g1_ankle_swing_example.cpp, accessible from here. For running instructions, refer to Quick Development.

To introduce the routine, first learn about the control modes of the G1 ankle joints.

G1 Robot Ankle Joint Control Modes

Parallel Structure Model: G1 Ankle Parallel Mechanism

The G1 ankle joint adopts a parallel mechanism and provides users with two control modes:

PR Mode: Controls the Pitch (P) and Roll (R) motors of the ankle joint (default mode, corresponding to the URDF model).
AB Mode: Directly controls the A and B motors of the ankle joint (requires users to calculate the parallel mechanism kinematics themselves).

Since there are two sets of constraint relationships between the PR and AB joint movements of the G1 robot ankle joint, the ankle joint actually has only two degrees of freedom. In hardware implementation, the AB joint is under active control (directly driven by motors), while the PR joint is in a passive state. To achieve PR Mode, we use indirect adjustment of the AB joint to control the PR joint.

The G1 robot's waist Pitch and Roll joints also use a parallel mechanism, providing two control modes: the PR mode and the AB mode.

Code Analysis

The following code is for the G1 29dof version model. Examples for other versions are also available in the unitree_sdk2 repository.

Core Types and Functions
Type Name	Description
DataBuffer	Multi-threaded data buffer tool class
ImuState	IMU related structure
MotorCommand	Motor command related structure
MotorState	Motor state related structure
Mode	G1 ankle joint control mode
G1JointIndex	Index of all joints sorted by name
G1Example	Core control logic class

The main program of the routine is defined in g1_ankle_swing_example.cpp, where the main program instantiates the G1Example class.

In the constructor of G1Example, two threads and a callback are created:

A low-level command sending thread that executes periodically, calling the function G1Example::LowCommandWriter at a default frequency of 500Hz.
A user-defined control logic thread, the motor control thread, calling the function G1Example::Control at a default frequency of 500Hz.
A low-level state receiving callback, calling the function G1Example::LowStateHandler.

The specific code is shown as follows:

G1Example(std::string networkInterface)
    : time_(0.0),
      control_dt_(0.002),
      duration_(3.0),
      mode_pr_(Mode::PR),
      mode_machine_(0) {
  ChannelFactory::Instance()->Init(0, networkInterface);
  // create publisher
  lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
  lowcmd_publisher_->InitChannel();
  // create subscriber
  lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
  lowstate_subscriber_->InitChannel(std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);
  // create threads
  command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1Example::LowCommandWriter, this);
  control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1Example::Control, this);
}
Low-Level Command Sending Function

The function G1Example::LowCommandWriter is used to periodically send low-level commands.

void LowCommandWriter() {
  LowCmd_ dds_low_command;
  dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
  dds_low_command.mode_machine() = mode_machine_;
  const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
  if (mc) {
    for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
      dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
      dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
      dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
      dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
      dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
      dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
    }
    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
    lowcmd_publisher_->Write(dds_low_command);
  }
}
Low-Level State Receiving Callback Function

When the low-level state message of the topic rt/lowstate is received, the program will automatically call the G1Example::LowStateHandler function. The callback function extracts the state of the motor and the state of the IMU. The structure of the low-level state can be referenced in Basic Services Interface.

void LowStateHandler(const void *message) {
  LowState_ low_state = *(const LowState_ *)message;
  if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
    std::cout << "[ERROR] CRC Error" << std::endl;
    return;
  }
  // get motor state
  MotorState ms_tmp;
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    ms_tmp.q.at(i) = low_state.motor_state()[i].q();
    ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
    if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
      std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
  }
  motor_state_buffer_.SetData(ms_tmp);
  // get imu state
  ImuState imu_tmp;
  imu_tmp.omega = low_state.imu_state().gyroscope();
  imu_tmp.rpy = low_state.imu_state().rpy();
  imu_state_buffer_.SetData(imu_tmp);
  // update mode machine
  if (mode_machine_ != low_state.mode_machine()) {
    if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
    mode_machine_ = low_state.mode_machine();
  }
}
User-Defined Control Logic Function

G1Example::Control implements:

Stage 1: Resetting the G1 robot from any initial joint position to the zero position (the first 3 seconds after the program starts);
Stage 2: Swinging the ankle joint in PR control mode for 3 seconds;
Stage 3: Swinging the ankle joint in AB control mode continuously until the program ends.
void Control() {
  ReportRPY();
  MotorCommand motor_command_tmp;
  const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    motor_command_tmp.tau_ff.at(i) = 0.0;
    motor_command_tmp.q_target.at(i) = 0.0;
    motor_command_tmp.dq_target.at(i) = 0.0;
    motor_command_tmp.kp.at(i) = Kp[i];
    motor_command_tmp.kd.at(i) = Kd[i];
  }
  if (ms) {
    time_ += control_dt_;
    if (time_ < duration_) {
      // [Stage 1]: set robot to zero posture
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
        motor_command_tmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
      }
    } else if (time_ < duration_ * 2) {
      // [Stage 2]: swing ankle using PR mode
      mode_pr_ = Mode::PR;
      double max_P = M_PI * 30.0 / 180.0;
      double max_R = M_PI * 10.0 / 180.0;
      double t = time_ - duration_;
      double L_P_des = max_P * std::sin(2.0 * M_PI * t);
      double L_R_des = max_R * std::sin(2.0 * M_PI * t);
      double R_P_des = max_P * std::sin(2.0 * M_PI * t);
      double R_R_des = -max_R * std::sin(2.0 * M_PI * t);
      motor_command_tmp.q_target.at(LeftAnklePitch) = L_P_des;
      motor_command_tmp.q_target.at(LeftAnkleRoll) = L_R_des;
      motor_command_tmp.q_target.at(RightAnklePitch) = R_P_des;
      motor_command_tmp.q_target.at(RightAnkleRoll) = R_R_des;
    } else {
      // [Stage 3]: swing ankle using AB mode
      mode_pr_ = Mode::AB;
      double max_A = M_PI * 30.0 / 180.0;
      double max_B = M_PI * 10.0 / 180.0;
      double t = time_ - duration_ * 2;
      double L_A_des = +max_A * std::sin(M_PI * t);
      double L_B_des = +max_B * std::sin(M_PI * t + M_PI);
      double R_A_des = -max_A * std::sin(M_PI * t);
      double R_B_des = -max_B * std::sin(M_PI * t + M_PI);
      motor_command_tmp.q_target.at(LeftAnkleA) = L_A_des;
      motor_command_tmp.q_target.at(LeftAnkleB) = L_B_des;
      motor_command_tmp.q_target.at(RightAnkleA) = R_A_des;
      motor_command_tmp.q_target.at(RightAnkleB) = R_B_des;
    }
    motor_command_buffer_.SetData(motor_command_tmp);
  }
}
Euler Angle Status Printing Function

G1Example::ReportRPY is used to periodically output Euler angle data.

void ReportRPY() {
  const std::shared_ptr<const ImuState> imu = imu_state_buffer_.GetData();
  if (imu) std::cout << "rpy: [" << imu->rpy.at(0) << ", " << imu->rpy.at(1) << ", " << imu->rpy.at(2) << "]" << std::endl;
}