# Inspire FTP Dexterity Hand

> 出典: https://support.unitree.com/home/en/G1_developer/inspire_ftp_dexterity_hand

Introduction to the Dexterous Hand SDK

The G1 can be equipped with the humanoid five-finger dexterous hand from Inspire Robotics. This dexterous hand features 6 degrees of freedom and 12 motion joints, allowing it to simulate complex human hand movements. The tactile version of this hand integrates 17 tactile sensors.

Control Methods

The dexterous hand offers two communication methods: Modbus RTU via RS-485 and Modbus TCP. For models without tactile sensors, only RS-485 is supported. This SDK supports both of the aforementioned methods for communication with the dexterous hand, forwarding data and control commands in DDS format.

The G1 provides a USB-to-serial module, allowing users to connect this USB module to the G1 development unit (PC2) for RS-485 communication control. The port is typically set to /dev/ttyUSB0 in this case, and you will need to run the program with the suffix 485.

Controlling with Inspire Official SDK

Users can write their own programs to control the dexterous hand according to the Inspire dexterous hand official communication protocol.

Controlling with Unitree Dexterous Hand SDK

The G1 communication is based on the DDS framework. To facilitate the use of unitree_sdk2 for controlling the dexterous hand, Unitree provides example programs that convert dexterous hand data into DDS messages.

SDK Interface Description

Users can send "inspire::inspire_hand_ctrl" messages to the topic "rt/inspire_hand/ctrl/*" to control the dexterous hand.
Messages of type "inspire::inspire_hand_state" can be received from the topic "rt/inspire_hand/state/*" to obtain the status of the dexterous hand.
Messages of type "inspire::inspire_hand_touch" can be received from the topic "rt/inspire_hand/touch/*" to obtain tactile sensor data.
Here, * is the topic suffix, which defaults to r, indicating the right hand.

When using 485 communication, the "rt/inspire_hand/touch/*" topic will not publish any messages.

rt/inspire_hand/ctrl/*
rt/inspire_hand/state/*
rt/inspire_hand/touch/*
user
G1

IDL Data Format

Motor data is organized in array format, containing data from the 12 motors of both hands. The specific formats for MotorCmd_.idl and MotorState_.idl can be found in the Basic Services Interface.

The dexterous hand data format is largely consistent with the official description document from Inspire. For more details, refer to the .idl files in inspire_hand_sdk/hand_idl.

// inspire_hand_ctrl.idl
module inspire
{
    struct inspire_hand_ctrl
    {
        sequence<int16,6>  pos_set;
        sequence<int16,6>  angle_set;
        sequence<int16,6>  force_set;
        sequence<int16,6>  speed_set;
        int8 mode;
    };
};

// inspire_hand_state.idl
module inspire
{
    struct inspire_hand_state
    {
        sequence<int16,6>  pos_act;
        sequence<int16,6>  angle_act;
        sequence<int16,6>  force_act;
        sequence<int16,6>  current;
        sequence<uint8,6>  err;
        sequence<uint8,6>  status;
        sequence<uint8,6>  temperature;
    };
};


// inspire_hand_touch.idl
module inspire
{
    struct inspire_hand_touch
    {
        sequence<int16,9>   fingerone_tip_touch;      // Pinky fingertip touch data
        sequence<int16,96>  fingerone_top_touch;      // Pinky fingertip touch data
        sequence<int16,80>  fingerone_palm_touch;     // Pinky palm touch data
        sequence<int16,9>   fingertwo_tip_touch;      // Ring fingertip touch data
        sequence<int16,96>  fingertwo_top_touch;      // Ring fingertip touch data
        sequence<int16,80>  fingertwo_palm_touch;     // Ring palm touch data
        sequence<int16,9>   fingerthree_tip_touch;    // Middle fingertip touch data
        sequence<int16,96>  fingerthree_top_touch;    // Middle fingertip touch data
        sequence<int16,80>  fingerthree_palm_touch;   // Middle palm touch data
        sequence<int16,9>   fingerfour_tip_touch;     // Index fingertip touch data
        sequence<int16,96>  fingerfour_top_touch;     // Index fingertip touch data
        sequence<int16,80>  fingerfour_palm_touch;    // Index palm touch data
        sequence<int16,9>   fingerfive_tip_touch;     // Thumb fingertip touch data
        sequence<int16,96>  fingerfive_top_touch;     // Thumb fingertip touch data
        sequence<int16,9>   fingerfive_middle_touch;  // Thumb middle touch data
        sequence<int16,96>  fingerfive_palm_touch;    // Thumb palm touch data
        sequence<int16,112> palm_touch;                // Palm touch data
    };
};

note

Control messages have added a mode option, which allows for the combination of control commands implemented in binary to specify the execution of instructions.

- mode 0: 0000 (no operation)
- mode 1: 0001 (angle)
- mode 2: 0010 (position)
- mode 3: 0011 (angle + position)
- mode 4: 0100 (force control)
- mode 5: 0101 (angle + force control)
- mode 6: 0110 (position + force control)
- mode 7: 0111 (angle + position + force control)
- mode 8: 1000 (speed)
- mode 9: 1001 (angle + speed)
- mode 10: 1010 (position + speed)
- mode 11: 1011 (angle + position + speed)
- mode 12: 1100 (force control + speed)
- mode 13: 1101 (angle + force control + speed)
- mode 14: 1110 (position + force control + speed)
- mode 15: 1111 (angle + position + force control + speed)  

Joint Order in IDL
Id	0	1	2	3	4	5
Joint	Hand
Pinky	Ring	Middle	Index	Thumb-bend	Thumb-rotation
SDK Installation and Usage

This SDK is primarily implemented in Python and relies on unitree_sdk2_python, while also utilizing PyQt5 and PyQtGraph for visualization.

Install Dependencies

clone the SDK working directory:

git clone https://github.com/NaCl-1374/inspire_hand_ws.git

Update submodules:

git submodule init  # Initialize submodules
git submodule update  # Update submodules to the latest version

Install unitree_sdk2_python:

cd unitree_sdk2_python
pip install -e .

note

ref: unitree_sdk2_python，Install the required dependencies, and proceed with subsequent operations after the unitree_sdk2_python test passes.

Install inspire_hand_sdk

cd ../inspire_hand_sdk
pip install -e .

If error when pip3 install -e . ref: unitree_sdk2_python,This error mentions that the cyclonedds path could not be found. First compile and install cyclonedds:

Usage
Dexterous Hand and Environment Configuration

First, configure the network for the device. The default IP for the is:left hand is 192.168.123.210,right hand is 192.168.123.211. The device running the driver must be on the same subnet as the dexterous hand. After configuration, execute ping 192.168.123.210 or ping 192.168.123.211 to check if communication is normal.

If you need to adjust the IP of the dexterous hand and other parameters, you can run the Dexterous Hand Configuration Panel in the usage example below. The panel will automatically read the device information in the current network. After modifying the parameters on the panel, you need to click Write Settings to send the parameters to the dexterous hand. The parameters will not take effect until you click Save Settings and restart Hand.

For configuration using RS-485 communication, the method is similar to TCP, allowing you to modify the device ID through the configuration panel. However, due to the limited bandwidth of RS-485, only one device can run on a bus at about 20 Hz, so usually only the port number needs to be changed.

note

To grant serial port access to a user in Ubuntu, you can add the user to the dialout group, which has permissions for serial devices (e.g., /dev/ttyS0 or /dev/ttyUSB0). Here’s how:

Check if the user is already in the dialout group:

groups $USER

Add the user to the dialout group (replace your_username with the actual username):

sudo usermod -aG dialout your_username

Log out and back in (or reboot) to apply the new permissions.

Verify the permissions: You can check the device permissions with:

ls -l /dev/ttyUSB0

After these steps, the user should have access to serial devices.

note

If you modify the IP, you need to change the following code in the relevant files to update the ip option to the new IP address. It is recommended to change it to the same 192.168.123.*** subnet as unitree_sdk2.

# inspire_hand_sdk/example/Vision_driver.py and inspire_hand_sdk/example/Headless_driver.py
handler=inspire_sdk.ModbusDataHandler(ip=inspire_hand_defaut.defaut_ip, LR='r', device_id=1)          

# inspire_hand_sdk/example/init_set_inspire_hand.py
window = MainWindow(ip=defaut_ip)

Here, the LR option is the parameter for the DDS message suffix *, which can be defined according to the device.

When running, be sure to modify the DDS working network card option; for details, refer to the G1 SDK Quick Development for DDS configuration.

The main communication interface definitions of the SDK are as follows: please configure according to the instructions.

class ModbusDataHandler:
    def __init__(self, data=data_sheet, history_length=100, network=None, ip=None, port=6000, device_id=1, LR='r', use_serial=False, serial_port='/dev/ttyUSB0', baudrate=115200, states_structure=None, initDDS=True, max_retries=5, retry_delay=2):
        """_summary_
        Calling self.read() in a loop reads and returns the data, and publishes the DDS message at the same time        
        Args:
            data (dict, optional): Tactile sensor register definition. Defaults to data_sheet.
            history_length (int, optional): Hand state history_length. Defaults to 100.
            network (str, optional): Name of the DDS NIC. Defaults to None.
            ip (str, optional): ModbusTcp IP. Defaults to None will use 192.1686.11.210.
            port (int, optional): ModbusTcp IP port. Defaults to 6000.
            device_id (int, optional): Hand ID. Defaults to 1.
            LR (str, optional): Topic suffix l or r. Defaults to 'r'.
            use_serial (bool, optional): Whether to use serial mode. Defaults to False.
            serial_port (str, optional): Serial port name. Defaults to '/dev/ttyUSB0'.
            baudrate (int, optional): Serial baud rate. Defaults to 115200.
            states_structure (list, optional): List of tuples for state registers. Each tuple should contain (attribute_name, start_address, length, data_type). If None ,will publish All Data 
            initDDS (bool, optional): Run ChannelFactoryInitialize(0),only need run once in all program
            max_retries (int, optional): Number of retries for connecting to Modbus server. Defaults to 3.
            retry_delay (int, optional): Delay between retries in seconds. Defaults to 2.
        Raises:
            ConnectionError: raise when connection fails after max_retries
        """

Here's the translated usage example in English:

Usage Examples

Below are instructions for several common usage examples:

Publishing Control Commands via DDS:

Run the following script to publish control commands:

python inspire_hand_sdk/example/dds_publish.py

Subscribing to the Status of the Dexterous Hand and Tactile Sensor Data, and Visualizing:

Run the following script to subscribe to the status and sensor data of the dexterous hand, and visualize the data:

python inspire_hand_sdk/example/dds_subscribe.py

Dexterous Hand DDS Driver (Headless Mode):

Use the following script for driving operations in headless mode:

python inspire_hand_sdk/example/Headless_driver.py

note

Programs with 485 indicate the use of RS-485 drivers, while those without it indicate the use of TCP drivers. The l/r/double suffixes represent single left/right and both hands driving simultaneously.

Dexterous Hand DDS Driver (Panel Mode):

Use the following script to enter panel mode and control the dexterous hand's DDS driver:

python inspire_hand_sdk/example/Vision_driver.py

Dexterous Hand Configuration Panel:

Run the following script to use the configuration panel for the dexterous hand:

python inspire_hand_sdk/example/init_set_inspire_hand.py

Publishing Control Commands via DDS in C++:

Run the following commands to compile and run the example program:

cd inspire_hand_sdk
mkdir build && cd build
cmake ..
make 
./hand_dds