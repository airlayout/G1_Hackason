# Arm Control Routine

> 出典: https://support.unitree.com/home/en/G1_developer/arm_control_routine

This routine implements upper limb control of the robot through the DDS interface of high-level operation control. The program source code can be located at example/g1/high_level/g1_arm_sdk_dds_example.cpp and can also be accessed here. The operation method is similar to "Quick Development", but there is no need to enter debugging mode.

The DDS interface is provided by the built-in motion service. When testing, it is recommended to suspend the robot and enter locked standing mode.

Routine analysis
Main logic

The program is triggered by keyboard input. Press the Enter key according to the command line text prompt. The program will execute the following contents in sequence:

Move the arm to the zero position within 5 seconds.

Open your arms horizontally within 5 seconds.

Lower your arms within 5 seconds.

Slowly exit the arm control of high-level motion services within 2 seconds.

Detailed analysis

In the routine, the DDS interface is first called to subscribe to the topic message of "rt/arm_sdk"

unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
    arm_sdk_publisher;
unitree_hg::msg::dds_::LowCmd_ msg;

arm_sdk_publisher.reset(
    new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
        kTopicArmSDK));
arm_sdk_publisher->InitChannel();

Then send the corresponding control instructions to the "rt/arm_sdk" topic to realize the upper limb control of the robot. The control flow in the routine consists of four parts:

Arm reset
Open arms horizontally
Put arms down
Exit control
Arm reset
// Wait for ENTER to be pressed
std::cout << "Press ENTER to init arms ...";
std::cin.get();

std::cout << "Initailizing arms ...";
float init_time = 5.0f; //reset time
//Number of control steps required to reach reset state
int init_time_steps = static_cast<int>(init_time / control_dt);

for (int i = 0; i < init_time_steps; ++i) {
    // increase weight
    weight += delta_weight;
    weight = std::clamp(weight, 0.f, 1.f); // Limit the weight value to no more than 1

      //Set the smooth increase and decrease weight of motor rotation
     // When the weight changes from 0 to 1, the motor will gradually transition from the current position to the desired position
    msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight * weight);

    // Set the upper limb motor position as the initial position
    for (int j = 0; j < init_pos.size(); ++j) {
      msg.motor_cmd().at(arm_joints.at(j)).q(init_pos.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
  }

  // Send control instructions to upper limb motors
  arm_sdk_publisher->Write(msg);

  // Delay by one control step
  std::this_thread::sleep_for(sleep_time);
}

std::cout << "Done!" << std::endl;

Additional Note
JointIndex::kNotUsedJoint of motor_cmd is the unused joint motor serial number. In the SDK, the value of this joint angular position is used as the transition weight of other motor rotations (weight in the above code). When weight changes from 0 to 1, the motor will gradually transition from the current position to the desired position. The faster the weight changes, the faster the motor's transition speed will be. Please refer to "Sport Motion Service Interface".

Open arms horizontally
// Wait for ENTER to be pressed
std::cout << "Press ENTER to start arm ctrl ..." << std::endl;
std::cin.get();


std::cout << "Start arm ctrl!" << std::endl;
float period = 5.f; // time to open arms
//The number of control steps required to complete the action
int num_time_steps = static_cast<int>(period / control_dt);

std::array<float, 10> current_jpos_des{};

// lift arms up
for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des
  for (int j = 0; j < init_pos.size(); ++j) {
       // Gradually increase the desired joint position to the joint position when the arm is open
       // target_pos is the joint position when the arm is opened
      current_jpos_des.at(j) += std::clamp(target_pos.at(j) - current_jpos_des.at(j), -max_joint_delta, max_joint_delta);
  }

  // Set upper limb joint positions
  for (int j = 0; j < init_pos.size(); ++j) {
      msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

  // Send control instructions to upper limb motors
  arm_sdk_publisher->Write(msg);

  // Delay by one control step
  std::this_thread::sleep_for(sleep_time);
}
Put your arms down
for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des
    for (int j = 0; j < init_pos.size(); ++j) {
      // Gradually reduce the desired joint position to the initial position
      current_jpos_des.at(j) += std::clamp(init_pos.at(j) - current_jpos_des.at(j), -max_joint_delta, max_joint_delta);
    }

    // Set upper limb joint positions
    for (int j = 0; j < init_pos.size(); ++j) {
      msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    // Send control instructions to upper limb motors
    arm_sdk_publisher->Write(msg);

    // Delay by one control step
    std::this_thread::sleep_for(sleep_time);
}
Exit control
// stop control
    std::cout << "Stoping arm ctrl ...";
    float stop_time = 2.0f;
    int stop_time_steps = static_cast<int>(stop_time / control_dt);
    
    for (int i = 0; i < stop_time_steps; ++i) {
        // increase weight
        weight -= delta_weight;
        weight = std::clamp(weight, 0.f, 1.f);
    
         //Set the smooth increase and decrease weight of motor rotation
         //The motor will gradually enter the free state
        msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
    
        // Send control instructions to upper limb motors
        arm_sdk_publisher->Write(msg);
    
        // Delay by one control step
        std::this_thread::sleep_for(sleep_time);
    }
    
      std::cout << "Done!" << std::endl;