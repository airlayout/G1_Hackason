# Lidar Services Interface

> 出典: https://support.unitree.com/home/en/G1_developer/lidar_services_interface

Lidar service interface
Software Service Version Requirements

Lidar Driver >= 1.0.0.5. If the built-in service version is low, please contact technical support to upgrade to the correct version.

notice

The lidar of G1 produced after April 2026 has been changed to 'Mid360s'. For self-developed users, LIVOX official Livox-SDK2 and livox_ros_river2 need to be updated. Address:Livox-SDK-Git

Basic Information

The Mid-360 lidar is located in the middle of the head of the G1 robot. The coordinate relationship between the IMU inside the Mid-360 and the radar is (0.011, 0.02329, -0.04412) (unit: meter), and there is no rotational transformation. It can be represented by the following transformation matrix.

[
1
	
0
	
0
	
0.011


0
	
1
	
0
	
0.02329


0
	
0
	
1
	
−
0.04412


0
	
0
	
0
	
1
]
	​

1
0
0
0
	​

0
1
0
0
	​

0
0
1
0
	​

0.011
0.02329
−0.04412
1
	​

	​


The position relationship of the lidar coordinate system relative to the robot coordinate system is (-0.0, 0.0, -0.47618), and the placement method is inverted. The pitch axis inclination of the Mid-360 lidar is -2.3degrees.

Data Interface

The emitted data are point cloud and IMU data in DDS format respectively; the point cloud topic is rt/utlidar/cloud_livox_mid360 with an output frequency of 10Hz and a frame_id of "livox_frame"; the IMU topic is rt/utlidar/imu_livox_mid360 with an output frequency of 200Hz.

Subscription Example
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/Imu_.hpp>

#define LIDAR_TOPIC "rt/utlidar/cloud_livox_mid360"
#define IMU_TOPIC "rt/utlidar/imu_livox_mid360"

using namespace unitree::robot;
using namespace unitree::common;

void LidarHandler(const void* msg)
{
    const sensor_msgs::msg::dds_::PointCloud2_* pc_data = (const sensor_msgs::msg::dds_::PointCloud2_*)msg;
    std::cout << "Lidar data received" << std::endl;
}

void ImuHandler(const void* msg)
{
    const sensor_msgs::msg::dds_::Imu_* imu_data = (const sensor_msgs::msg::dds_::Imu_*)msg;
    std::cout << "Imu data received" << std::endl;
}

int main()
{
    ChannelFactory::Instance()->Init(0);
  

    ChannelSubscriberPtr<sensor_msgs::msg::dds_::PointCloud2_> lidar_sub_ = ChannelSubscriberPtr<sensor_msgs::msg::dds_::PointCloud2_>(new ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>(LIDAR_TOPIC));
    ChannelSubscriberPtr<sensor_msgs::msg::dds_::Imu_> imu_sub_ = ChannelSubscriberPtr<sensor_msgs::msg::dds_::Imu_>(new ChannelSubscriber<sensor_msgs::msg::dds_::Imu_>(IMU_TOPIC));


    lidar_sub_->InitChannel(std::bind(LidarHandler, std::placeholders::_1), 1);
    imu_sub_->InitChannel(std::bind(ImuHandler, std::placeholders::_1), 1);


    while (true)
    {
        sleep(10);
    }

    return 0;
}