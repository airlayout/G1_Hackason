# Basic Services Interface

> 出典: https://support.unitree.com/home/en/G1_developer/basic_services_interface

The low-level communication facilitates data interaction between the user PC and the robot. It uses the DDS protocol (DDS related knowledge can be found in 《DDS Communication Interface》).

Subscribe to the topic rt/lowstate (type: unitree_hg::msg::dds_::LowState_) to obtain the current state of the G1.

Publish to the topic rt/lowcmd (type: unitree_hg::msg::dds_::LowCmd_) to control all body joint motors (excluding the dexterous hand), the battery, and other devices.

To use the Dex3-1 force-controlled dexterous hand, publish to the topic rt/dex3/(left or right)/cmd (type: unitree_hg::msg::dds_::HandCmd_) to control the dexterous hand, and subscribe to the topic rt/dex3/(left or right)/state (type: unitree_hg::msg::dds_::HandState_) to receive the hand's state.

Interface Description

Subscribe to or publish topics using the methods described in the DDS communication interface. Topic data is stored in structures defined by IDL, with commonly used structures including:

Structure Name	Description
HandCmd_	Dex3-1 Control
HandState_	Dex3-1 Status
IMUState_	G1 IMU Status
LowCmd_	G1 Low-level Control
LowState_	G1 Low-level Status
MotorCmd_	G1 Motor Control
MotorState_	G1 Motor Status
PressSensorState_	Dex3-1 Tactile Feedback
Introduction to message types
Dex3-1 Control

unitree_hg::msg::dds_::HandCmd_

struct HandCmd_ {
  sequence<unitree_hg::msg::dds_::MotorCmd_> motor_cmd;  // Commands for all motors in the dexterous hand
};
Dex3-1 Status

unitree_hg::msg::dds_::HandState_

struct HandState_ {
  sequence<unitree_hg::msg::dds_::MotorState_> motor_state; // States of all motors in the dexterous hand
  unitree_hg::msg::dds_::IMUState_ imu_state;                // IMU state of the dexterous hand
  sequence<unitree_hg::msg::dds_::PressSensorState_> press_sensor_state; // Pressure sensor states
  float power_v;                                             // Power voltage of the dexterous hand
  float power_a;                                             // Power current of the dexterous hand
  unsigned long reserve[2];                                  // Reserved
};
IMU Status

unitree_hg::msg::dds_::IMUState_

struct IMUState_ {
  float quaternion[4];                                       // Quaternion QwQxQyQz
  float gyroscope[3];                                        // Gyroscope (angular velocity) omega_xyz
  float accelerometer[3];                                    // Acceleration acc_xyz
  float rpy[3];                                              // Euler angles
  short temperature;                                         // IMU temperature
};
Low-level Control

unitree_hg::msg::dds_::LowCmd_

struct LowCmd_ {
  octet mode_pr;                                             // Parallel mechanism (ankle and waist) control mode (default 0) 0:PR, 1:AB
  octet mode_machine;                                        // G1 Type：4：23-Dof;5:29-Dof;6:27-Dof(29Dof Fitted at the waist)
  unitree_hg::msg::dds_::MotorCmd_ motor_cmd[35];            // Control commands for all body motors
  unsigned long reserve[4];                                  // Reserved
  unsigned long crc;                                         // Checksum
};
Low-level Status

unitree_hg::msg::dds_::LowState_

struct LowState_ {
  octet mode_pr;                                             // Parallel mechanism (ankle and waist) control mode (default 0) 0:PR, 1:AB
  octet mode_machine;                                        // G1 Type
  unsigned long tick;                                        // Timer incrementing every 1ms
  unitree_hg::msg::dds_::IMUState_ imu_state;                // IMU state
  unitree_hg::msg::dds_::MotorState_ motor_state[35];        // States of all body motors
  octet wireless_remote[40];                                 // Raw data from Unitree physical remote control
  unsigned long reserve[4];                                  // Reserved
  unsigned long crc;                                         // Checksum
};
Motor Control

unitree_hg::msg::dds_::MotorCmd_

struct MotorCmd_ {
  octet mode;                                                // Motor control mode 0:Disable, 1:Enable
  float q;                                                   // Target joint position
  float dq;                                                  // Target joint velocity
  float tau;                                                 // Feedforward torque
  float kp;                                                  // Joint stiffness coefficient
  float kd;                                                  // Joint damping coefficient
  unsigned long reserve[3];                                  // Reserved
};
Motor Status

unitree_hg::msg::dds_::MotorState_

struct MotorState_ {
  octet mode;                                                // Current motor mode
  float q;                                                   // Joint feedback position (rad)
  float dq;                                                  // Joint feedback velocity (rad/s)
  float ddq;                                                 // Joint feedback acceleration (rad/s^2)
  float tau_est;                                             // Joint feedback torque   
  float q_raw;                                               // Reserved
  float dq_raw;                                              // Reserved
  float ddq_raw;                                             // Reserved
  short temperature[2];                                      // Motor temperature (surface and winding)
  unsigned long sensor[2];                                   // Sensor data
  float vol;                                                 // Motor terminal voltage
  unsigned long motorstate;                                  // Motor state
  unsigned long reserve[4];                                  // Reserved
};
Dex3-1 Tactile Feedback

unitree_hg::msg::dds_::PressSensorState_

struct PressSensorState_ {
  float pressure[12];
  float temperature[12];
};