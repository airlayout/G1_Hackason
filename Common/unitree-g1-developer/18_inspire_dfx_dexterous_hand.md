# Inspire DFX Dexterous Hand

> 出典: https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand

DFX Dexterous Hand

Unitree G1 can be equipped with Inspire Robotics (RH56DFX)'s dexterous hand, which has 6 degrees of freedom and 12 motion joints to mimic the human hand for complex movements.

Control Method

The official SDK of Inspire Hand communicates through a serial port. H1 provides a USB to serial module, which can be plugged into the H1 Development Computing Unit (PC2) for communication and control of the Dexterous Hand. The port is usually named /dev/ttyUSB1(left hand)、/dev/ttyUSB2(right hand).

Control with Inspire SDK

Users can write their own program to control the dexterous hand according to the official communication protocol of Inspire Dexterous Hand.

Control with Unitree Inspire Hand SDK

G1 communication is built on top of the DDS framework. In order to use unitree_sdk2 to control the dexterous hand. Unitree provides a sample program that converts data sent and received through the serial port into DDS messages (see the bottom of the document for the download link).

G1 Hand Interface

The user can control the hand by publish the "unitree_go::msg::dds::MotorCmds_" message to the topic "rt/inspire/cmd", and get the hand state by subscribe the "unitree_go::msg::dds::MotorStates_" message from the topic "rt/inspire/state".

rt/inspire/cmd
rt/inspire/state
user
G1

IDL Message Type

Motor data in array format, containing 12 motor data for both hands.
For the details of the MotorCmds_.idl and MotorState_.idl, see Basic_Services_Interface

note

Currently the dexterous hand only supports joint control, i.e. only the parameter q makes sense in the idl format. The others are reserved.

# namespace unitree_go::msg::dds_

# unitree_go::msg::dds_::MotorCmds_
struct MotorCmds_ 
{
    sequence<unitree_go::msg::dds_::MotorCmd_> cmds;
};

# unitree_go::msg::dds_::MotorStates_
struct MotorCmds_ 
{
    sequence<unitree_go::msg::dds_::MotorState> states;
};
Joint Motor Sequence
Id	0	1	2	3	4	5	6	7	8	9	10	11
Joint	Right Hand	Left Hand
pinky	ring	middle	index	thumb-bend	thumb-rotation	pinky	ring	middle	index	thumb-bend	thumb-rotation
Examples

The following is the encapsulated dexterous hand control interface from the example.

/**
 * @brief Unitree H1 Hand Controller
 * The user can subscribe to "rt/inspire/state" to get the current state of the hand and publish to "rt/inspire/cmd" to control the hand.
 * 
 *                  IDL Types
 * user ---(unitree_go::msg::dds_::MotorCmds_)---> "rt/inspire/cmd"
 * user <--(unitree_go::msg::dds_::MotorStates_)-- "rt/inspire/state"
 * 
 * @attention Currently the hand only supports position control, which means only the `q` field in idl is used.
 */
class H1HandController
{
public:
    H1HandController()
    {
        this->InitDDS_();
    }

    /**
     * @brief Control the hand to a specific label
     */
    void ctrl(std::string label)
    {
        if(labels.find(label) != labels.end())
        {
            this->ctrl(labels[label], labels[label]);
        }
        else
        {
            std::cout << "Invalid label: " << label << std::endl;
        }
    }

    /**
     * @brief Move the fingers to the specified angles
     * 
     * @note The angles should be in the range [0, 1]
     *       0: close  1: open
     */
    void ctrl(
        const Eigen::Matrix<float, 6, 1>& right_angles, 
        const Eigen::Matrix<float, 6, 1>& left_angles)
    {
        for(size_t i(0); i<6; i++)
        {
            cmd.cmds()[i].q() = right_angles(i);
            cmd.cmds()[i+6].q() = left_angles(i);
        }
        handcmd->Write(cmd);
    }

    /**
     * @brief Get the right hand angles
     * 
     * Joint order: [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    Eigen::Matrix<float, 6, 1> getRightQ()
    {
        std::lock_guard<std::mutex> lock(mtx);
        Eigen::Matrix<float, 6, 1> q;
        for(size_t i(0); i<6; i++)
        {
            q(i) = state.states()[i].q();
        }
        return q;
    }

    /**
     * @brief Get the left hand angles
     * 
     * Joint order: [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    Eigen::Matrix<float, 6, 1> getLeftQ()
    {
        std::lock_guard<std::mutex> lock(mtx);
        Eigen::Matrix<float, 6, 1> q;
        for(size_t i(0); i<6; i++)
        {
            q(i) = state.states()[i+6].q();
        }
        return q;
    }

    unitree_go::msg::dds_::MotorCmds_ cmd;
    unitree_go::msg::dds_::MotorStates_ state;
private:
    void InitDDS_()
    {
        handcmd = std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/inspire/cmd");
        handcmd->InitChannel();
        cmd.cmds().resize(12);
        handstate = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(
            "rt/inspire/state");
        handstate->InitChannel([this](const void *message){
            std::lock_guard<std::mutex> lock(mtx);
            state = *(unitree_go::msg::dds_::MotorStates_*)message;
        });
        state.states().resize(12);
    }

    // DDS parameters
    std::mutex mtx;
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::MotorCmds_> handcmd;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::MotorStates_> handstate;

    // Saved labels
    std::unordered_map<std::string, Eigen::Matrix<float, 6, 1>> labels = {
        {"open",   Eigen::Matrix<float, 6, 1>::Ones()},
        {"close",  Eigen::Matrix<float, 6, 1>::Zero()},
        {"half",   Eigen::Matrix<float, 6, 1>::Constant(0.5)},
    };
};
Package Download
Date	Update Description	Download Address
2024-5-7	Initial Release(Reuse the H1 routine)	Click to download
2025-8-28		github