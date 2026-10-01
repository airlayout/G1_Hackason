# Arm Action Inferface

> 出典: https://support.unitree.com/home/en/G1_developer/arm_action_interface

Arm Action Service Interface

Unitree has added many interactive functions based on upper limb actions for the G1, managed by the Arm Action Service. The main function of the Arm Action Service Interface (Arm Action Client) is to call these upper limb interaction functions.

Note

The Arm Action Service Interface relies on the built-in motion control. After entering debug mode, the built-in motion control exits completely, and the Arm Action Service becomes invalid.
Please ensure the surrounding environment is safe when using the Arm Action Service Interface, especially when triggering the playback of taught actions.

RPC Interface Call Method

Users can create a unitree::robot::g1::ArmAction object and send requests through its member functions. The built-in Arm Action Service responds to the requests and executes the operations.

#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_api.hpp>
Action ID

Action IDs are updated periodically. Use Get Action List to see the actions available in the current firmware version.

Action	ID
Recover Initial Arm Pose	99
Double Hand Flying Kiss	11
Single Hand Flying Kiss	12
Arms Horizontal	15
Applause	17
High Five	18
Hug	19
Double Hand Heart	20
Single Hand Heart	21
Double Hand Cross	22
Right Hand Horizontal	23
Dynamic Light Wave	24
Wave Hand in Front Chest	25
Wave Hand High	26
Handshake	27
Interface Introduction
Function Name	Execute Action
Function Prototype	int32_t ExecuteAction(int32_t action_id)
Function Overview	Execute a preset arm interaction action.
Parameters	id: Arm Action ID.
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	Blocking execution. Refer to the previous "Action ID" section for Arm Action IDs. This description applies when the input parameter is of type int.
Function Name	Execute Action
Function Prototype	int32_t ExecuteAction(const std::string &action_name)
Function Overview	Execute a pre-recorded taught action.
Parameters	action_name: The saved name of the taught action.
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	Non-blocking execution. The action name recorded by the user using the teaching function in the App, case-sensitive. This description applies when the input parameter is of type string.
Function Name	Stop Custom Action
Function Prototype	int32_t StopCustomAction()
Function Overview	Stop executing the taught action.
Parameters	None
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	After stopping, the arm returns to the initial position.
Function Name	Get Action List
Function Prototype	int32_t GetActionList(std::string &data)
Function Overview	List the currently available actions.
Parameters	data: Action list (includes usable actions, special action requirements for FsmID, names and durations of taught actions)
Return Value	Returns the callable action list if successful, otherwise returns an error code.
Remarks	Subject to the response from the server.
Error Codes
Error Code	Meaning	Remarks
7400	Topic is occupied	An action is being executed
7401	Arm is raised. Please use Action ID 99 or the previously used same ID to recover the arm to the initial state.	Applicable to sustained actions like Arms Horizontal, Heart, etc.
7402	Action ID does not exist	
7404	Current FsmID cannot trigger this action.	Some actions cannot be triggered under walking/running motion control.
Turning Off the Arm Action Service

If you need to independently develop upper limb actions via the /arm_sdk topic, you must first turn off Unitree's built-in Arm Control Service.

You can turn off Unitree's built-in Arm Control Service through the Device State Service Interface. The service name for the Arm Action Service is: g1_arm_example.

Example:

#include <unitree/robot/b2/robot_state/robot_state_client.hpp>
#include <unitree/common/time/time_tool.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

int main(int32_t argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: turn_off_arm_example [NetWorkInterface(eth0)]" << std::endl;
        exit(0);
    }

    std::string networkInterface = "eth0", serviceName = "g1_arm_example";

    if (argc > 1)
    {
        networkInterface = argv[1];
    }

    std::cout << "NetWorkInterface:" << networkInterface << std::endl;
    std::cout << "Switch ServiceName:" << serviceName << std::endl; 

    ChannelFactory::Instance()->Init(0, networkInterface);

    RobotStateClient rsc;
    rsc.SetTimeout(10.0f);
    rsc.Init();

    std::string clientApiVersion = rsc.GetApiVersion();
    std::string serverApiVersion = rsc.GetServerApiVersion();

    if (clientApiVersion != serverApiVersion)
    {
        std::cout << "client and server api versions are not equal." << std::endl;
    }
    int status = 0;
    while (1)
    {
        int ret = rsc.ServiceSwitch(serviceName, 0, status);
        if (ret == 0 && status != 0)
        {
            std::cout << "Call ServiceSwitch[" << serviceName << ",0] ret:" << ret << ", status:" << status << std::endl;
            break;
        }
    }
    ChannelFactory::Instance()->Release();
    return 0;
}