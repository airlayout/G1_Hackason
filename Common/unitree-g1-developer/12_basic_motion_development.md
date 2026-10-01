# Basic Motion Development

> 出典: https://support.unitree.com/home/en/G1_developer/basic_motion_development

Basic motion development refers to the process of developing and debugging the underlying control system of a robot.

In Basic motion development, developers need to write control algorithms and driver programs to achieve precise control of the robot's joint motors. This includes controlling parameters such as motor speed, position, and torque to enable the robot's motion and posture adjustments. Basic motion development often requires a deep understanding of the robot's hardware architecture and control system to ensure accurate and reliable motion control.

note

Under the current system version, once the G1 is turned on, the built-in motion control program will automatically start, even if you do not operate the remote control. The program periodically sends commands with a speed of 0. However, if you use the SDK in this state, you may cause conflicting instructions and thus cause G1 to jitter.

Therefore, when you need to use the SDK for development debugging, make sure that G1 is in debug mode to stop the motion control program from sending instructions, so as to avoid potential instruction conflict issues. You can press L2+A to confirm that you are in debug mode.

If the behavior after pressing L2 + A does not match with the teaching video, you can press L2 + R2 several times to ensure entering the debugging mode.