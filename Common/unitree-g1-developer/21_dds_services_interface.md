# DDS Services Interface

> 出典: https://support.unitree.com/home/en/G1_developer/dds_services_interface

Introduction to Unitree SDK

unitree_sdk2 It is a layer of encapsulation on DDS that supports QoS configuration of DDS components, provides a simple encapsulation interface for application development, and implements an RPC mechanism based on DDS Topic. It is suitable for data communication in different scenarios through publish/subscribe and request/response methods between internal and external processes in G1.

Introduction to DDS

DDS (Data Distribution Service) is a middleware that is a distributed communication specification published by OMG. It adopts a publish/subscribe model and provides various QoS service quality policies to ensure real-time, efficient, and flexible data distribution, meeting the needs of various distributed real-time communication applications.

(Image source: https://www.dds-foundation.org)

Explanation:

unitree_sdk2 The installation and configuration tutorial will not be repeated in this article. Please refer to the 《quick_development》 section.

Interface Description
unitree::robot::ChannelFactory

unitree::robot::ChannelFactory provides a single instance of ChannelFactory under Unitree:: robot to create a DDS Topic based Channel. ChannelFactory must call the initialization interface before use to initialize the underlying DDS configuration. The calling method is as follows
unitree::robot::ChannelFactory::Instance()->Init(0)

The descriptions of each interface are as follows:

Function Name	Instance
Function Prototype	static unitree::robot::ChannelFactory* unitree::robot::ChannelFactory::Instance()
Functional Overview	Obtain a singleton static pointer
Parameters	None
Return Value	Unitree::robot::ChannelFactory singleton static pointer
Remarks	
Function Name	Init
Function Prototype	void Init(int32_t domainId, const std::string& networkInterface = "", bool enableSharedMemory = false)
Functional Overview	Initialize ChannelFactory by specifying three initialization parameters: Domain Id, network card name, and whether to use shared memory.
Parameters	domainId: Default domain id for constructing DdsParticipant；
networkInterface: Specify the network card name, default to blank;
enableSharedMemory: Whether to use shared memory, default to false.
Return Value	None
Remarks	If the networkInterface is empty, automatic selection of network card configuration will be generated; When developing applications outside of G1, enableSharedMemory needs to be set to false.
Function Name	CreateSendChannel
Function Prototype	template ChannelPtr CreateSendChannel(const std::string& name)
Functional Overview	Specify name to create a Channel with a specified message type of MSG for sending data.
Parameters	name：Channel Name.
Return Value	template ChannelPtr
Remarks	
Function Name	CreateRecvChannel
Function Prototype	template ChannelPtr CreateRecvChannel(const std::string& name, std::function<void(const void*)> callback, int32_t queuelen = 0)
Functional Overview	Specify name to create a Channel with a specified message type of MSG for receiving data.
Parameters	name：Channel Name；
callback: callback function when the message arrives;
queuelen: The length of the message cache queue; If the length is 0, do not enable columns in the message cache.
Return Value	template ChannelPtr
Remarks	If the message takes a long time to process in the callback function, it is recommended to enable the message cache queue to avoid blocking the DDS callback thread.
Function Name	Release
Function Prototype	void Release()
Functional Overview	Release ChannelFactory static resources.
Parameters	None
Return Value	None
Remarks	unitree::robot::ChannelFactory::Instance()->Release()
unitree::robot::ChannelPublisher

unitree::robot::ChannelPublisher Class implements the message publishing function of the specified type

Class name	Constructors and Destructors
template ChannelPublisher	explicit ChannelPublisher(const std::string& channelName); ~ChannelPublisher()

Explanation:

unitree::robot::ChannelPublisher Used during construction unitree::robot::ChannelFactory::Instance()->CreateSendChannel(mChannelName)，So when using unitree::robot::ChannelPublisher Before that, you need to initialize the unitree::robot::ChannelFactory singleton.

Function Name	InitChannel
Function Prototype	void InitChannel()
Functional Overview	Initialize the Channel and prepare it for sending messages.
Parameters	None
Return Value	None
Remarks	
Function Name	CloseChannel
Function Prototype	void CloseChannel()
Functional Overview	Close Channel
Parameters	None
Return Value	None
Remarks	
Function Name	Write
Function Prototype	bool Write(const MSG& msg)
Functional Overview	Send a specified type of message.
Parameters	msg: Send a message of type MSG.
Return Value	true: Successfully sent; false: Sending failed.
Remarks	
unitree::robot::ChannelSubscriber

unitree::robot::ChannelSubscriber Implemented the specified type of message subscription function.

Class Name	Constructors and Destructors
template ChannelSubscriber	explicit ChannelSubscriber(const std::string& channelName); ~ChannelSubscriber()

Attention:

unitree::robot::ChannelSubscriber Used during construction unitree::robot::ChannelFactory::Instance()->CreateSendChannel(mChannelName)，So when using unitree::robot::ChannelSubscriber initialization is required before proceeding unitree::robot::ChannelFactory singleton。

Function Name	InitChannel
Function Prototype	void InitChannel(const std::function<void(const void*)>& callback, int64_t queuelen = 0)
Functional Overview	Initialize the Channel and prepare it for receiving or processing messages.
Parameters	callback: callback function when the message arrives;
queuelen: The length of the message cache queue; If the length is 0, do not enable columns in the message cache.
Return Value	None
Remarks	If the message takes a long time to process in the callback function, it is recommended to enable the message cache queue to avoid blocking the DDS callback thread.
Function Name	CloseChannel
Function Prototype	void CloseChannel()
Functional Overview	Close Channel。
Parameters	None
Return Value	None
Remarks	
Function Name	GetLastDataAvailableTime
Function Prototype	int64_t GetLastDataAvailableTime()
Functional Overview	Gets the last time a message was received.
Parameters	None
Return Value	Returns -1 when the Channel is not initialized; Otherwise, it returns a timestamp that monotonically increments from 0 starting from system startup, accurate to microseconds.
Remarks	
Service Client Interface

Multiple service components inside the robot provide some functional interfaces externally through RPC Client, and users can easily control the robot through the interface of the Client.

List of some general error codes
Number	Error description	Remarks
3001	Unknown error	Client/server returned
3102	Request sending error	Client returned
3103	API not registered	Client returned
3104	Request timeout	Client returned
3105	Request and response data do not match	Client returned
3106	Invalid response data	Client returned
3107	Invalid lease	Client returned
3201	Response sending error	Occurred on the server and will not be returned to the client
3202	Internal server error	Server returned
3203	API not implemented on the server	Server returned
3204	API parameter error	Server returned
3205	Request rejected	Server returned
3206	Invalid lease	Server returned
3207	Lease already exists	Server returned
Partial Client Common Interface Description
Function Name	Init
Function Prototype	void Init()
Functional Overview	Client initialization, completing logic such as client API registration.
Parameters	None
Return Value	None
Remarks	
Function Name	SetTimeout
Function Prototype	void SetTimeout(float timeout)
Functional Overview	Set the RPC request timeout time.
Parameters	timeout: in seconds.
Return Value	None
Remarks	If no timeout is set, the default timeout time is 1 second.
Function Name	WaitLeaseApplied
Function Prototype	void WaitLeaseApplied()
Functional Overview	Applying for a lease, the function will block until the lease is applied for.
Parameters	None
Return Value	None
Remarks	Only valid when lease is enabled.
Function Name	GetApiVersion
Function Prototype	const std::string& GetApiVersion()
Functional Overview	Obtain the client API version.
Parameters	None
Return Value	Returns the client API version.
Remarks	
Function Name	GetApiVersion
Function Prototype	const std::string& GetServerApiVersion()
Functional Overview	Obtain the server API version.
Parameters	None
Return Value	Returns the server API version.
Remarks	
Message publishing example

Routine path: unitree_sdk2/example/helloworld/publisher.cpp

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>                                  
#include "HelloWorldData.hpp"

#define TOPIC "TopicHelloWorld"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    /*
     * Init with defalue domain 0.
     */
    ChannelFactory::Instance()->Init(0);

    /*
     * New ChannelPublisherPtr
     */
    ChannelPublisherPtr<HelloWorldData::Msg> publisher = ChannelPublisherPtr<HelloWorldData::Msg>(new ChannelPublisher<HelloWorldData::Msg>(TOPIC));

    /*
     * Init channel
     */
    publisher->InitChannel();

    while (true)
    {
        /*
         * Send message
         */
        HelloWorldData::Msg msg(unitree::common::GetCurrentTimeMillisecond(), "HelloWorld.");
        publisher->Write(msg);
        sleep(1);
    }   

    return 0;
}
Message subscription example

routine path： unitree_sdk2/example/helloworld/subscriber.cpp

#include <unitree/robot/channel/channel_subscriber.hpp>
#include "HelloWorldData.hpp"

#define TOPIC "TopicHelloWorld"

using namespace unitree::robot;
using namespace unitree::common;

void Handler(const void* msg)
{
    const HelloWorldData::Msg* pm = (const HelloWorldData::Msg*)msg;

    std::cout << "userID:" << pm->userID() << ", message:" << pm->message() << std::endl;
}

int main()
{
    /*
     * Init with defalue domain 0.
     */
    ChannelFactory::Instance()->Init(0);
  
    /*
     * New ChannelSubscriberPtr
     */
    ChannelSubscriberPtr<HelloWorldData::Msg> subscriber = ChannelSubscriberPtr<HelloWorldData::Msg>(new ChannelSubscriber<HelloWorldData::Msg>(TOPIC));

    /*
     * Init channel
     */
    subscriber->InitChannel(std::bind(Handler, std::placeholders::_1), 1);

    sleep(5);
  
    /*
     * Close channel
     */
    subscriber->CloseChannel();

    std::cout << "reseted. sleep 3" << std::endl;

    sleep(3);

    /*
     * Init channel use last input parameter.
     */
    subscriber->InitChannel();

    /*
     * Loop wait message.
     */
    while (true)
    {
        sleep(10);
    }

    return 0;
    }
G1 List of common topics
TopicName	Type Group	Message Type Definition	Info
rt/dex3/left/state	hg	HandState_	Get left dexterous hand feedback state - low-frequency mode
rt/lf/dex3/left/state	hg	HandState_	Get left dexterous hand feedback state
rt/dex3/left/cmd	hg	HandCmd_	Control left dexterous hand
rt/dex3/right/state	hg	HandState_	Get left dexterous hand feedback state
rt/lf/dex3/right/state	hg	HandState_	Get right dexterous hand feedback state - low-frequency mode
rt/dex3/right/cmd	hg	HandCmd_	Control right dexterous hand
rt/lf/mainboardstate	hg	MainBoardState_	Get mainboard feedback information
rt/lowstate	hg	LowState_	Get low-level feedback information (IMU, motors, etc.)
rt/lf/lowstate	hg	LowState_	Get low-level feedback information (IMU, motors, etc.) - low-frequency mode
rt/lowcmd	hg	LowCmd_	Low-level control commands
rt/lf/bmsstate	hg	BmsState_	Get battery feedback data
rt/odommodestate	go2	IMUState_	Get odometry information
rt/lf/odommodestate	go2	IMUState_	Get odometry information - low-frequency mode
rt/lf/secondary_imu	hg	IMUState_	Get body IMU data - low-frequency mode
rt/secondary_imu	hg	IMUState_	Get body IMU data

Note

We do not provide IDL files directly. Instead, we provide the message definition files compiled by the Cyclone DDS idlc component. Please refer to the following:

Unitree-sdk2 IDL
PythonSDk2 IDL
ROS2 MSG