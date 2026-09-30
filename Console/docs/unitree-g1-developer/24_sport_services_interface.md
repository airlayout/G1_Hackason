# Sport Services Interface

> 出典: https://support.unitree.com/home/en/G1_developer/sport_services_interface

High-level motion service interface

The main functions of the high-level motion service interface are robot complete machine control and upper limb control. The whole machine control is basically equivalent to the remote control operation. The upper limb control can independently control the upper limb motor to complete the operation task while running the robot's built-in mobile controller. High-level services provide two interfaces, RPC and DDS.

note

Note: The high-level motion service depends on the built-in operation control. After entering the debugging mode, the built-in operation control is completely exited and the high-level motion service becomes invalid.

RPC interface
Calling method

Users can create unitree::robot::g1::LocoClient objects, send requests through member functions, and the built-in high-level motion service responds to the request and performs operations.

#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
    
unitree::robot::g1::LocoClient client;
Interface introduction
Generic interface
function name	Damp
Function prototype	int32_t Damp()
Function overview	The robot enters damping mode.
Parameter	None.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The success of the final state switch depends on the built-in operation control state switching logic, please refer to the previous section 'Mode Switching'.
function name	Start
Function prototype	int32_t Start()
Function overview	By switching modes, enter the main operation control.
Parameter	None.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The success of the final state switch depends on the built-in operation control state switching logic, please refer to the previous section 'Mode Switching'.
function name	Squat
Function prototype	int32_t Squat()
Function overview	By switching modes, enter squat mode.
Parameter	None.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The success of the final state switch depends on the built-in operation control state switching logic, please refer to the previous section 'Mode Switching'.
function name	Sit
Function prototype	int32_t Sit()
Function overview	By switching modes, enter the sit mode.
Parameter	None.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The success of the final state switch depends on the built-in operation control state switching logic, please refer to the previous section 'Mode Switching'.
function name	StandUp
Function prototype	int32_t StandUp()
Function overview	By switching modes, enter the stand up mode.
Parameter	None.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The success of the final state switch depends on the built-in operation control state switching logic, please refer to the previous section 'Mode Switching'.
function name	ZeroTorque
Function prototype	int32_t ZeroTorque()
Function overview	By switching modes, enter the zero torque mode.
Parameter	None.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The success of the final state switch depends on the built-in operation control state switching logic, please refer to the previous section 'Mode Switching'.
function name	Move
Function prototype	int32_t Move(float vx, float vy, float vyaw)
Function overview	Send speed command.
Parameter	vx: forward speed; vy: horizontal speed; vyaw: Rotation speed.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The speed command takes effect for 1 second by default, and the behavior can be modified through the ContinuousGait function.
function name	Move
Function prototype	int32_t Move(float vx, float vy, float vyaw, bool continous_move)
Function overview	Send speed command.
Parameter	vx: forward speed; vy: horizontal speed; vyaw: Rotation speed; continous_move：Whether it continues to be effective.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	When continus_move=true, the speed command takes effect for a long time; On the contrary, it only takes effect for 1 second.
function name	StopMove
Function prototype	int32_t StopMove()
Function overview	Set the speed command to zero.
Parameter	None.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	None.
function name	SwitchMoveMode
Function prototype	int32_t ContinuousGait(bool flag)
Function overview	Modifies the behavior of Move(float vx, float vy, float vyaw).
Parameter	flag: Set true to enable the Move() continuous response mode. In motion mode, it will always respond to the latest Move command. Set false to disable the Move() continuous response mode. If no new Move command is received, it will automatically stop after a delay of 1s.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The continuous response mode is turned off by default. For safety reasons, it is recommended to enable the continuous response mode with caution.
function name	BalanceStand
Function prototype	int32_t BalanceStand()
Function overview	Enter the balanced standing mode of the main control.
Parameter	None.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	In the balanced standing mode, when the speed command is 0 and the robot is stable, the robot stops stepping and steps only when necessary.
function name	ContinuousGait
Function prototype	int32_t ContinuousGait(bool flag)
Function overview	Keep moving.
Parameter	flag: Set true to enable, false to disable.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	After starting the continuous gait, the robot dog will continue to step even if the current speed is 0.
Expert interface
ID	Mode	Remarks
0	Zero Torque	No Balance Control
1	Damping	No Balance Control
2	Position Control Squat	No Balance Control
3	Position Control Sit Down	No Balance Control
4	Lock Standing	No Balance Control
706	Balance Squat, Squat Stand	\
702	Lie Down, Stand Up	\
500	Walk Motion	\
501	Walk Motion-3Dof-waist	\
801	Run	The 29dof device ai_sport was updated to version 802 after version 8.6.x.x
function name	GetFsmId
Function prototype	int32_t GetFsmId(int& fsm_id)
Function overview	Get the operation control mode.
Parameter	fsm_id：Used to receive the current mode id.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	See Mode ID Description for details.
function name	SetFsmId
Function prototype	int32_t SetFsmId(int fsm_id)
Function overview	Set the operation control mode.
Parameter	fsm_id: target mode id.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	For details, see Mode ID Description and Mode Switching Description.
function name	GetFsmMode
Function prototype	int32_t GetFsmMode(int& fsm_mode)
Function overview	Get whether the main transport control is in standing state.
Parameter	fsm_mode: used for receiving status.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	0: standing state; 1: moving state.
function name	SetVelocity
Function prototype	int32_t SetVelocity(float vx, float vy, float omega, float duration = 1.f)
Function overview	Set the speed.
Parameter	vvx: forward speed; vy: horizontal speed; omega: rotation speed; duration: duration of speed command.
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The program will automatically set the cropping to the allowed range.
function name	SetSpeedMode
Function prototype	int32_t SetSpeedMode(int speed_mode)
Function overview	Adjust the maximum speed in running mode.
Parameter	speed_mode 0:1.0m/s;1:2.0m/s;2:2.7m/s;3:3.0m/s
Return value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	

User Development Mode

In certain scenarios, there is a need to quickly switch between a custom robot full-body controller and Unitree's built-in motion control. In such cases, the switching interface that only allows transitions between debug mode and motion control mode cannot meet the requirements.

Therefore, Unitree provides an interface that temporarily switches the robot into debug mode, allowing a custom controller to take over the robot and then exit flexibly. We refer to this as User Development Mode.

In this mode, users can send low-level commands to the motors using the topic rt/user_lowcmd. At this point, full control of all motors is handed over to the user.

However, please note that if you need to switch from walking/running mode to your own motion control, make sure that both the first and last actions of your motion control are in a standard standing posture! Otherwise, the robot may lose control.

The methods for entering and exiting User Development Mode are as follows:

Function Name	SwitchToUserCtrl
Function Prototype	int32_t SwitchToUserCtrl()
Overview	Switches from the current mode to User Development Mode
Parameters	None
Return Value	Returns 0 on success; otherwise returns the relevant error code.
Remarks	None
Function Name	SwitchToInternalCtrl(InternalFsmMode mode)
Function Prototype	int32_t SwitchToInternalCtrl(InternalFsmMode mode)
Overview	Switches from User Development Mode back to internal motion control mode
Parameters	InternalFsmMode includes:
unitree::robot::g1::InternalFsmMode::LAST — the motion control mode before entering User Development Mode
unitree::robot::g1::InternalFsmMode::PASSIVE — damping mode
unitree::robot::g1::InternalFsmMode::WALKRUN — walk/run mode
Return Value	Returns 0 on success; otherwise returns the relevant error code.
Remarks	None
DDS interface

The DDS interface supports upper limb control and can only be used in Locked Stance, Movement Control 1 and Movement Control 2.

Calling method

Users can send LowCmd type messages to the rt/arm_sdk topic based on "DDS Services Interface" and the underlying service "Basic Service Interface". Refer to "Joint Motor Sequence" to set motor commands for the upper limbs.

Interface introduction

The following table shows the meaning of each valid element in LowCmd_.motor_cmd_, and the rest of the elements are invalid.

Index	Meaning
29	MotorCmd_.q represents the weight, and the value range is [0.0, 1.0].
12 - 28	Waist and upper limb motor control Parameter.
SportModeState Interface

After the firmware version 1.5.1 update, you can obtain the current state of G1 through the topic rt/sportmodestate. It includes:

Attribute	Description	Format
fsm_id	Mode ID, see the "Expert Interface" section above	unsigned long
fsm_mode	Mode status. Includes:
0: Static, allows switching to other modes
1: Dynamic, switching to most modes is not allowed	unsigned long
task_id	Upper limb interaction action ID; for details, see Arm Action Service Interface	unsigned long
task_time	Execution time of upper limb actions.
Unit: seconds, increments from 0 until the action is completed.
When the action is a handshake, this value remains constant during the holding period.	float

Explanation of Mode States

What are static states and dynamic states? What is the difference between them?

According to expert interfaces, G1 can currently switch between various modes. However, to ensure safety, we have imposed restrictions on mode switching while the robot is in motion.

When the robot's current state/posture is unsuitable for mode switching, we prohibit the robot from changing modes.

For example, during continuous stepping in climbing mode, switching to any mode other than damping mode is not allowed, as such a transition could risk imbalance. By monitoring this field, you can determine whether the robot is suitable for switching at that moment.

Damping mode, as the ultimate fallback, can always be activated.

The message format for SportModeState is:

// SportModeState_.idl
module unitree_hg {
    module msg {
        module dds_ {
            struct SportModeState_ {
                unsigned long fsm_id;
                unsigned long fsm_mode;
                unsigned long task_id;
                float task_time;
            };
        };
    };
};