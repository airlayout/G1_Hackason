# Architecture Description

> 出典: https://support.unitree.com/home/en/G1_developer/architecture_description

System architecture diagram

G1 will connect to the cloud service only after user authorization

cloud service

Cloud services have several main functions:

Collect G1 operating data (not involving privacy), conduct fault detection and statistics.

Help users achieve remote operations, mainly through WebRTC. Image and control traffic can be forwarded point-to-point or through the turn server, which is not collected or analyzed by the server.

System upgrade iteration.

Communication for cloud services:

mqtt is used to establish IoT communication with each device and is responsible for monitoring faults, system upgrades, and transmitting WebRTC signaling.

The http service connects the App and Web front-end to establish a binding relationship between the user and the robot.

The turn/stun server is used to facilitate WebRTC point-to-point connections and provide server data forwarding when point-to-point connections cannot be achieved.

G1

The OTA module communicates with the cloud server through mqtt and is responsible for uploading fault information, system upgrades, and forwarding WebRTC signaling.

The WebRTC module implements the main data pipeline with the App, including audio and video streams, radar point clouds, motion status and control instructions.

The Bluetooth (BLE) part is used to establish contact with the App and is mainly used for network configuration and security verification.

The communication between each functional module is mainly implemented by DDS. DDS IDL is compatible with ROS2 (you need to select the adapted RMW). The EDU version can call the interface through DDS or ROS2.

Sensor data such as motors and radars are collected through the serial port and then forwarded to the DDS middle layer.

G1 edu has 2 built-in computing units. PC1 is dedicated to the Unitree motion control program and is not open to the public. Developers can only use PC2 for secondary development. The IP addresses for PC2 is 192.168.123.164. Please contact our support engineers for the initial user name and password.

App

User management module, connecting to Yushu management platform through HTTP Web API. Responsible for binding robot, WebRTC connection establishment and other functions.

G1’s Bluetooth module is used to configure the network.

WebRTC module, the main data traffic is realized through WebRTC, including image transmission, point cloud, motion status and control command issuance.

Development interface

DDS, supports C++ and Python.

ROS2 interface.

GST, for image transmission only.