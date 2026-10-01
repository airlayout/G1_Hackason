# VuiClient Service Interface

> 出典: https://support.unitree.com/home/en/G1_developer/VuiClient_Service

Introduction

The audio and lighting-related hardware facilities embedded in the upper body of G1 enable the robot to achieve voice interaction capabilities,

including:

RGB light strip - 256 colors
Speaker - 8Ω 3W (5W peak)
Four - microphone array. microphone spacing of 20mm (quasi - linear, silicon microphones)
Component Location
Secondary Development

Software service version requirement:
Vui_Service >= 2.0.3.8， Vui Module>= 2.0.0.3. If the built - in service version is low, please contact technical support to upgrade to the correct version.

The audio and lighting - related interfaces currently mainly provide the following capabilities. Users can use the following combinations to develop their own voice interaction programs.

ASR (Automatic Speech Recognition), default non - streaming model, local offline. Can include azimuth, emotion, speaker role and other information.
TTS (Text - to - Speech), local offline. Pronunciation role is selectable, currently only supports Chinese.
Audio stream control
Volume control
Light strip color control
AudioClient Class

The AudioClient class can implement functions such as text - to - speech, audio control/playback, and lighting control.

Interface List:
Function Name	TtsMaker
Function Prototype	int32_t TtsMaker(const std::string& text, int32_t speaker_id)
Function Overview	Text - to - speech conversion
Parameters	text: Text
speaker_id: Role ID
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	speaker_id 0 for Chinese roles and 1 for English roles. Mixed Chinese and English modes are not supported.
speaker_id 0	你好。我是宇树科技的机器人。例程启动成功
speaker_id 1	Hello. I'm a robot from Unitree Robotics. The example has started successfully. 
Function Name	GetVolume
Function Prototype	int32_t GetVolume(uint8_t &volume）
Function Overview	Get the system volume
Parameters	volume: Volume level (0 - 100)
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	
Function Name	SetVolume
Function Prototype	int32_t SetVolume(uint8_t volume)
Function Overview	Set the system volume
Parameters	volume: Volume level (0 - 100)
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	
Function Name	LedControl
Function Prototype	int32_t LedControl(uint8_t R, uint8_t G, uint8_t B)
Function Overview	Light strip control
Parameters	R: Red (0 - 255)
G: Green (0 - 255)
B: Blue (0 - 255)
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	The interval between calls to this interface must be greater than 200ms.
Function Name	PlayStream
Function Prototype	int32_t PlayStream(std::string app_name, std::string stream_id, std::vector<uint8_t> pcm_data)
Function Overview	Audio stream playback
Parameters	app_name: Application name
stream_id: Identification ID, the same ID means continuous playback from cache, different IDs mean interrupting the current playback
pcm_data: PCM format, sampling rate 16K, single - channel, 16 - bit
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	Please pay attention to the audio format.
Function Name	PlayStop
Function Prototype	int32_t PlayStop(std::string app_name)
Function Overview	Stop playback
Parameters	app_name: Application name
Return Value	Returns 0 if the call is successful, otherwise returns the relevant error code.
Remarks	
ASR Messages / Audio Play State

When the robot's microphone is turned on (switch to the wake - up mode in the APP or remote control), the built - in microphone + ASR module will recognize the human voice in the environment.

Subscribe to the topic rt/audio_msg (class type: std_msgs::msg::dds_::String_) to obtain the recognition information provided by the built-in offline ASR module.

{
    "index": 1,
    "timestamp":29319303490
    "text": "Hello",
    "angle": 90,
    "speaker_id": 0,
    "sense": "unknown",
    "confidence": 0.95,
    "language": "en - US",
    "is_final": true
}
Parameter Name	Parameter Type	Meaning
index	Integer	Unique message sequence number
timestamp	Integer	Timestamp
text	String	Speech recognition result
angle	Integer	Azimuth angle 0 - 180
speaker_id	Integer	Speaker recognition result
sense	String	Emotion recognition result
confidence	Float	Confidence level
language	String	Language type
is_final	Integer	End flag (used in the streaming recognition mode, non - streaming by default)

Play State

{
   "play_state": 1
}
Parameter Name	Parameter Type	Meaning
play_state	Integer	0:play stop 1:start play
#include <fstream>
#include <iostream>
#include <thread>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#include "wav.hpp"

#define AUDIO_FILE_PATH "../example/g1/audio/test.wav"
#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"
#define GROUP_IP "239.168.123.161"
#define PORT 5555

#define WAV_SECOND 5 // record seconds
#define WAV_LEN (16000 * 2 * WAV_SECOND)
int sock;

void asr_handler(const void *msg) {
  std_msgs::msg::dds_::String_ *resMsg = (std_msgs::msg::dds_::String_ *)msg;
  std::cout << "Topic:\"rt/audio_msg\" recv: " << resMsg->data() << std::endl;
}

std::string get_local_ip_for_multicast() {
  struct ifaddrs *ifaddr, *ifa;
  char host[NI_MAXHOST];
  std::string result = "";

  getifaddrs(&ifaddr);
  for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
      if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
      getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      std::string ip(host);
      if (ip.find("192.168.123.") == 0) {
          result = ip;
          break;
      }
  }
  freeifaddrs(ifaddr);
  return result;
}

void thread_mic(void) {
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(PORT);
  local_addr.sin_addr.s_addr = INADDR_ANY;
  bind(sock, (sockaddr *)&local_addr, sizeof(local_addr));

  ip_mreq mreq{};
  inet_pton(AF_INET, GROUP_IP, &mreq.imr_multiaddr);
  std::string local_ip = get_local_ip_for_multicast();
  std::cout << "local ip: "<<local_ip << std::endl;
  mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());
  setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

  int total_bytes = 0;
  std::vector<int16_t> pcm_data;
  pcm_data.reserve(WAV_LEN / 2);
  std::cout << "start record!" << std::endl;
  while (total_bytes < WAV_LEN) {
    char buffer[2048];
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if (len > 0) {
      size_t sample_count = len / 2;
      const int16_t *samples = reinterpret_cast<const int16_t *>(buffer);
      pcm_data.insert(pcm_data.end(), samples, samples + sample_count);
      total_bytes += len;
    }
  }

  WriteWave("record.wav", 16000, pcm_data.data(), pcm_data.size(), 1);
  std::cout << "record finish! save to record.wav " << std::endl;
}

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: audio_client_example [NetWorkInterface(eth0)]"
              << std::endl;
    exit(0);
  }
  int32_t ret;
  /*
   * Initilaize ChannelFactory
   */
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);

  /*ASR message Example*/
  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      AUDIO_SUBSCRIBE_TOPIC);
  subscriber.InitChannel(asr_handler);

  /*Volume Example*/
  uint8_t volume;
  ret = client.GetVolume(volume);
  std::cout << "GetVolume API ret:" << ret
            << "  volume = " << std::to_string(volume) << std::endl;
  ret = client.SetVolume(100);
  std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

  /*TTS Example*/
  ret = client.TtsMaker("你好。我是宇树科技的机器人。例程启动成功",
                        0);  // Auto play
  std::cout << "TtsMaker API ret:" << ret << std::endl;
  unitree::common::Sleep(5);

  ret = client.TtsMaker(
      "Hello. I'm a robot from Unitree Robotics. The example has started "
      "successfully. ",
      1);  // Engilsh TTS
  std::cout << "TtsMaker API ret:" << ret << std::endl;
  unitree::common::Sleep(8);

  /*Audio Play Example*/
  int32_t sample_rate = -1;
  int8_t num_channels = 0;
  bool filestate = false;
  std::vector<uint8_t> pcm =
      ReadWave(AUDIO_FILE_PATH, &sample_rate, &num_channels, &filestate);

  std::cout << "wav file sample_rate = " << sample_rate
            << " num_channels =  " << std::to_string(num_channels)
            << " filestate =" << filestate << std::endl;

  if (filestate && sample_rate == 16000 && num_channels == 1) {
    client.PlayStream(
        "example", std::to_string(unitree::common::GetCurrentTimeMillisecond()),
        pcm);
    std::cout << "start play stream" << std::endl;
    unitree::common::Sleep(3);
    std::cout << "stop play stream" << std::endl;
    ret = client.PlayStop("example");
  } else {
    std::cout << "audio file format error, please check!" << std::endl;
  }

  /*LED Control Example*/
  client.LedControl(0, 255, 0);
  unitree::common::Sleep(1);
  client.LedControl(0, 0, 0);
  unitree::common::Sleep(1);
  client.LedControl(0, 0, 255);

  std::cout << "AudioClient api test finish , asr start..." << std::endl;

  std::thread mic_t(thread_mic);

  while (1) {
    sleep(1);  // wait for asr message
  }
  mic_t.join();
  return 0;
}