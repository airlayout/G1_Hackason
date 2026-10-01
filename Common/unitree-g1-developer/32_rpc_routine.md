# RPC Routine

> 出典: https://support.unitree.com/home/en/G1_developer/rpc_routine

RPC Example

This example uses the high-level RPC interface to switch the robot mode, as well as control standing and moving.

note

ai_sport >= 8.2.0.0 version is LOCO_SERVICE_NAME = "sport";
Lower than this version LOCO_SERVICE_NAME = "loco".

The source code can be found in unitree_sdk2/example/g1/high_level/g1_loco_client_example.cpp, and can also be accessed here. The operation method is similar to 《Quick Development》, but there is no need to enter the debug mode.

Example Analysis
Main Logic

Different control effects can be achieved by specifying startup parameters at startup.

Parameter description
Parameter	Description	Assignment example
--network_interface	Specify the name of the communication network card	enp3s0
--get_fsm_id	Get the state machine id of the robot's upper controller	/
--get_fsm_mode	Get the state machine mode of the robot's upper controller	/
--get_phase	Get the phase of the robot	/
--set_fsm_id	Set the state machine id	1
--set_velocity	Set the movement speed [vx vy omega duration (optional)]	"0.5 0 0 1"
--damp	Enter damping mode	/
--start	Enter main movement control	/
--squat	Squat	/
--sit	Sit down	/
--stand_up	Stand	/
--zero_torque	Zero torque mode	/
--stop_move	Stop movement	/
--low_stand	Stand low	/
--balance_stand	Balanced stand	/
--continous_gait	Continuous gait	true
--switch_move_mode	Switch movement mode	true
--move	Move at a certain speed [vx vy omega]	"0.5 0 0"
--set_speed_mode	Set the maximum speed under running control	0/1/2/3
Startup example

Move 
1
𝑠
1s at a speed of 
0.5
𝑚
/
𝑠
0.5m/s in the 
𝑥
x direction

./g1_loco_client --network_interface=enp3s0 --set_velocity="0.5 0 0 1"
Code analysis
Parsing parameters

If the parameter has a value, the format is --${key}=${value} or --${key}="${value}";

If the parameter has no value, the format is --${key}

std::map<std::string, std::string> args = {{"network_interface", "lo"}};

  std::map<std::string, std::string> values;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.substr(0, 2) == "--") {
      size_t pos = arg.find("=");
      std::string key, value;
      if (pos != std::string::npos) {
        key = arg.substr(2, pos - 2);
        value = arg.substr(pos + 1);

        if (value.front() == '"' && value.back() == '"') {
          value = value.substr(1, value.length() - 2);
        }
      } else {
        key = arg.substr(2);
        value = "";
      }
      if (args.find(key) != args.end()) {
        args[key] = value;
      } else {
        args.insert({{key, value}});
      }
    }
  }
Initialize dds communication instance
unitree::robot::ChannelFactory::Instance()->Init(0, args["network_interface"]);
Initialize the upper-level control client
unitree::robot::g1::LocoClient client;

  client.Init();
  client.SetTimeout(10.f);
Response command

Parse the command line parameters one by one and implement the upper-level control of the robot by calling the corresponding API.

for (const auto &arg_pair : args) {
    std::cout << "Processing command: [" << arg_pair.first << "] with param: ["
              << arg_pair.second << "] ..." << std::endl;
    if (arg_pair.first == "network_interface") {
      continue;
    }

    if (arg_pair.first == "get_fsm_id") {
      int fsm_id;
      client.GetFsmId(fsm_id);
      std::cout << "current fsm_id: " << fsm_id << std::endl;
    }

    if (arg_pair.first == "get_fsm_mode") {
      int fsm_mode;
      client.GetFsmMode(fsm_mode);
      std::cout << "current fsm_mode: " << fsm_mode << std::endl;
    }

    if (arg_pair.first == "get_balance_mode") {
      int balance_mode;
      client.GetBalanceMode(balance_mode);
      std::cout << "current balance_mode: " << balance_mode << std::endl;
    }

    if (arg_pair.first == "get_swing_height") {
      float swing_height;
      client.GetSwingHeight(swing_height);
      std::cout << "current swing_height: " << swing_height << std::endl;
    }

    if (arg_pair.first == "get_stand_height") {
      float stand_height;
      client.GetStandHeight(stand_height);
      std::cout << "current stand_height: " << stand_height << std::endl;
    }

    if (arg_pair.first == "get_phase") {
      std::vector<float> phase;
      client.GetPhase(phase);
      std::cout << "current phase: (";
      for (const auto &p : phase) {
        std::cout << p << ", ";
      }
      std::cout << ")" << std::endl;
    }

    if (arg_pair.first == "set_fsm_id") {
      int fsm_id = std::stoi(arg_pair.second);
      client.SetFsmId(fsm_id);
      std::cout << "set fsm_id to " << fsm_id << std::endl;
    }

    if (arg_pair.first == "set_balance_mode") {
      int balance_mode = std::stoi(arg_pair.second);
      client.SetBalanceMode(balance_mode);
      std::cout << "set balance_mode to " << balance_mode << std::endl;
    }

    if (arg_pair.first == "set_swing_height") {
      float swing_height = std::stof(arg_pair.second);
      client.SetSwingHeight(swing_height);
      std::cout << "set swing_height to " << swing_height << std::endl;
    }

    if (arg_pair.first == "set_stand_height") {
      float stand_height = std::stof(arg_pair.second);
      client.SetStandHeight(stand_height);
      std::cout << "set stand_height to " << stand_height << std::endl;
    }

    if (arg_pair.first == "set_velocity") {
      std::vector<float> param = stringToFloatVector(arg_pair.second);
      auto param_size = param.size();
      float vx, vy, omega, duration;
      if (param_size == 3) {
        vx = param.at(0);
        vy = param.at(1);
        omega = param.at(2);
        duration = 1.f;
      } else if (param_size == 4) {
        vx = param.at(0);
        vy = param.at(1);
        omega = param.at(2);
        duration = param.at(3);
      } else {
        std::cerr << "Invalid param size for method SetVelocity: " << param_size
                  << std::endl;
        return 1;
      }

      client.SetVelocity(vx, vy, omega, duration);
      std::cout << "set velocity to " << arg_pair.second << std::endl;
    }

    if (arg_pair.first == "damp") {
      client.Damp();
    }

    if (arg_pair.first == "start") {
      client.Start();
    }

    if (arg_pair.first == "squat") {
      client.Squat();
    }

    if (arg_pair.first == "sit") {
      client.Sit();
    }

    if (arg_pair.first == "stand_up") {
      client.StandUp();
    }

    if (arg_pair.first == "zero_torque") {
      client.ZeroTorque();
    }

    if (arg_pair.first == "stop_move") {
      client.StopMove();
    }

    if (arg_pair.first == "high_stand") {
      client.HighStand();
    }

    if (arg_pair.first == "low_stand") {
      client.LowStand();
    }

    if (arg_pair.first == "balance_stand") {
      client.BalanceStand();
    }

    if (arg_pair.first == "continous_gait") {
      bool flag;
      if (arg_pair.second == "true") {
        flag = true;
      } else if (arg_pair.second == "false") {
        flag = false;
      } else {
        std::cerr << "invalid argument: " << arg_pair.second << std::endl;
        return 1;
      }
      client.ContinuousGait(flag);
    }

    if (arg_pair.first == "switch_move_mode") {
      bool flag;
      if (arg_pair.second == "true") {
        flag = true;
      } else if (arg_pair.second == "false") {
        flag = false;
      } else {
        std::cerr << "invalid argument: " << arg_pair.second << std::endl;
        return 1;
      }
      client.SwitchMoveMode(flag);
    }

    if (arg_pair.first == "move") {
      std::vector<float> param = stringToFloatVector(arg_pair.second);
      auto param_size = param.size();
      float vx, vy, omega;
      if (param_size == 3) {
        vx = param.at(0);
        vy = param.at(1);
        omega = param.at(2);
      } else {
        std::cerr << "Invalid param size for method SetVelocity: " << param_size
                  << std::endl;
        return 1;
      }
      client.Move(vx, vy, omega);
    }
  }