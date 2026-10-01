# Odometer Service Interface

> 出典: https://support.unitree.com/home/en/G1_developer/odometer_service_interface

Odometry Service Interface is used to publish the robot's position, speed, Euler angle, yaw angular velocity and quaternion information. The publishing frequency includes high frequency (500Hz) and low frequency (20Hz)

Software Service Version Requirements

State Estimator >= 1.0.0.1. If the built-in service version is lower, please contact technical support to upgrade to the correct version.

Interface Description

Users can receive odometry's information at a frequency of 500Hz by subscribing to the DDS topic "rt/odommodestate", and obtain odometry's information at a frequency of 20Hz by subscribing to the DDS topic "rt/lf/odommodestate". High-frequency and low-frequency information can be received at the same time.

The applicable range of the odometer:

This odometer relies on the motion control system and is part of the high-level interface; it cannot be used in development mode (low-level).

Description of Odometry's information

Coordinate system definition:
World coordinate system:————The world coordinate system is established at the ground projection point of the robot's base center. The x-axis is towards the front direction of the base, while y-axis and z-axis are towards the left and upstraight direction respectively. The x, y, and z axes obey the right-hand rule distribution.
Robot coordinate system:————The robot coordinate system is established at the robot's base center. The orientation of x, y, and z axes is aligned with the world coordinate system.

1.Position information
The position information is stored in a three-dimensional array, of which the members are successively positions of the robot's base center along the x, y and z axes of the world coordinate system, with the unit of meter.

2.Velocity information
The velocity information is stored in a three-dimensional array, of which the members are successively velocities of the robot's base center along the x, y and z axes of the world coordinate system, with the unit of meter per second.

3.Euler angle information
The Euler angle information is stored in a three-dimensional array, of which the members are successively rotations of the robot coordinate system around the x, y and z axes of the world coordinate system, with the unit of rad.

4.Yaw angular velocity information
The yaw angular velocity information is rotation angular velocity of the body around the z-axis of the robot coordinate system, with the unit of rad per second.

5.Quaternion information
The quaternion information is stored in a four-dimensional array, which represents the rotating situation of the robot coordinate system around the world coordinate system. The members of the array are successively w, x, y, and z, and w^2 + x^2 + y^2 + z^2 = 1.

Example code
#include <iostream>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_SPORT_STATE "rt/odommodestate"//high frequency
#define TOPIC_SPORT_LF_STATE "rt/lf/odommodestate"//low frequency

class Custom
{
public:
    explicit Custom()
    {}

    ~Custom()
    {}

    void Init();

private:

    /*high frequency message handler function for subscriber*/
    void HighFreOdomMessageHandler(const void* messages);
    /*low frequency message handler function for subscriber*/
    void LowFreOdomMessageHandler(const void* messages);

private:

    unitree_go::msg::dds_::SportModeState_ estimator_state{};
    unitree_go::msg::dds_::SportModeState_ lf_estimator_state{};

    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> estimate_state_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> lf_estimate_state_subscriber;
};


void Custom::Init()
{
    /*create subscriber*/
    estimate_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_SPORT_STATE));
    estimate_state_subscriber->InitChannel(std::bind(&Custom::HighFreOdomMessageHandler, this, std::placeholders::_1), 1);

    /*create subscriber*/
    lf_estimate_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_SPORT_LF_STATE));
    lf_estimate_state_subscriber->InitChannel(std::bind(&Custom::LowFreOdomMessageHandler, this, std::placeholders::_1), 1);
}

void Custom::HighFreOdomMessageHandler(const void* message)
{
    estimator_state = *(unitree_go::msg::dds_::SportModeState_*)message;

    std::cout << "position info: " << std::endl;
    std::cout << "x: " << estimator_state.position()[0] << std::endl;
    std::cout << "y: " << estimator_state.position()[1] << std::endl;
    std::cout << "z: " << estimator_state.position()[2] << std::endl;

    std::cout << "velocity info: " << std::endl;
    std::cout << "x: " << estimator_state.velocity()[0] << std::endl;
    std::cout << "y: " << estimator_state.velocity()[1] << std::endl;
    std::cout << "z: " << estimator_state.velocity()[2] << std::endl;

    std::cout << "eular angle info: " << std::endl;
    std::cout << "x: " << estimator_state.imu_state().rpy()[0] << std::endl;
    std::cout << "y: " << estimator_state.imu_state().rpy()[1] << std::endl;
    std::cout << "z: " << estimator_state.imu_state().rpy()[2] << std::endl;

    std::cout << "yaw speed info: " << std::endl;
    std::cout << estimator_state.yaw_speed() << std::endl;

    std::cout << "Quaternion info: " << std::endl;
    std::cout << "w: " << estimator_state.imu_state().quaternion()[0] << std::endl;
    std::cout << "x: " << estimator_state.imu_state().quaternion()[1] << std::endl;
    std::cout << "y: " << estimator_state.imu_state().quaternion()[2] << std::endl;
    std::cout << "z: " << estimator_state.imu_state().quaternion()[3] << std::endl;
}

void Custom::LowFreOdomMessageHandler(const void* message)
{
    lf_estimator_state = *(unitree_go::msg::dds_::SportModeState_*)message;

    std::cout << "position info: " << std::endl;
    std::cout << "x: " << lf_estimator_state.position()[0] << std::endl;
    std::cout << "y: " << lf_estimator_state.position()[1] << std::endl;
    std::cout << "z: " << lf_estimator_state.position()[2] << std::endl;

    std::cout << "velocity info: " << std::endl;
    std::cout << "x: " << lf_estimator_state.velocity()[0] << std::endl;
    std::cout << "y: " << lf_estimator_state.velocity()[1] << std::endl;
    std::cout << "z: " << lf_estimator_state.velocity()[2] << std::endl;

    std::cout << "eular angle info: " << std::endl;
    std::cout << "x: " << lf_estimator_state.imu_state().rpy()[0] << std::endl;
    std::cout << "y: " << lf_estimator_state.imu_state().rpy()[1] << std::endl;
    std::cout << "z: " << lf_estimator_state.imu_state().rpy()[2] << std::endl;

    std::cout << "yaw speed info: " << std::endl;
    std::cout << lf_estimator_state.yaw_speed() << std::endl;

    std::cout << "Quaternion info: " << std::endl;
    std::cout << "w: " << lf_estimator_state.imu_state().quaternion()[0] << std::endl;
    std::cout << "x: " << lf_estimator_state.imu_state().quaternion()[1] << std::endl;
    std::cout << "y: " << lf_estimator_state.imu_state().quaternion()[2] << std::endl;
    std::cout << "z: " << lf_estimator_state.imu_state().quaternion()[3] << std::endl;
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    Custom custom;
    custom.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}