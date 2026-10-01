# Motion Switcher Service Interface

> 出典: https://support.unitree.com/home/en/G1_developer/motion_witcher_service_interface

MotionSwitcherClient class

MotionSwitcherClient is a client provided by the motion control switching service. Through MotionSwitcherClient, users can conveniently release the G1 motion control mode via RPC and enter the debug mode developed by the user.
Using the MotionSwitcherClient class via #include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

List of interface error codes
Error number	Error description	Remarks
7001	Request parameter error.	Server return
7002	Service busy, retry again pelease.	Server return
7004	Unsupport mode name。	Server return
7005	Internal command execute error.	Server return
7006	Check command execute error.	Server return
7007	Switch command execute error.	Server return
7008	Release command execute error.	Server return
7009	Costomize config set error.	Server return
Class and interface description
Class Name	Creation and Deconstruction
MotionSwitcherClient	explicit MotionSwitcherClient();
~MotionSwitcherClient();
Function Name	CheckMode
Function Prototype	int32_t CheckMode(std::string& form, std::string& name)
Function Summary	Detects the current form and motion control mode.
Parameters	form: The current Go2 form. "0" for Standard Form; "1" for Wheel-Foot Form <BR> name: Motion control mode name. Please refer to "Motion Control Mode Name" above for details.
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Notes	
Function Name	SelectMode
Function Prototype	int32_t SelectMode(const std::string& name)
Function Summary	Selects the motion control mode.
Parameters	name: Motion control mode name.
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Notes	
Function Name	ReleaseMode
---	---
Function Prototype	int32_t ReleaseMode()
Function Overview	Release motion mode.
Parameter	None
Return Value	If the call is successful, 0 will be returned. Otherwise, relevant error codes will be returned.
Remarks	