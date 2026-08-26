#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/client/client.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

namespace {

constexpr char kServiceName[] = "slam_operate";
constexpr char kApiVersion[] = "1.0.0.1";
constexpr char kSlamInfoTopic[] = "rt/slam_info";
constexpr char kSlamKeyInfoTopic[] = "rt/slam_key_info";

constexpr std::int32_t kStopNode = 1901;
constexpr std::int32_t kStartMapping = 1801;
constexpr std::int32_t kEndMapping = 1802;

std::string json_escape(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

class SlamClient final : public unitree::robot::Client {
 public:
  SlamClient() : Client(kServiceName, false) {
    info_subscriber_ = std::make_shared<
        unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>(
        kSlamInfoTopic);
    key_info_subscriber_ = std::make_shared<
        unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>(
        kSlamKeyInfoTopic);
    info_subscriber_->InitChannel(
        [this](const void* message) { on_status(message, kSlamInfoTopic); }, 1);
    key_info_subscriber_->InitChannel(
        [this](const void* message) { on_status(message, kSlamKeyInfoTopic); }, 1);
  }

  void Init() override {
    SetApiVersion(kApiVersion);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(kStopNode);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(kStartMapping);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(kEndMapping);
  }

  void initialize(float timeout_seconds) {
    Init();
    SetTimeout(timeout_seconds);
  }

  bool wait_for_status(std::chrono::seconds timeout) {
    std::unique_lock<std::mutex> lock(status_mutex_);
    return status_condition_.wait_for(lock, timeout,
                                      [this] { return received_status_; });
  }

  int call_start() {
    return call(kStartMapping, R"({"data":{"slam_type":"indoor"}})");
  }

  int call_stop(const std::string& map_path) {
    const std::string parameters =
        "{\"data\":{\"address\":\"" + json_escape(map_path) + "\"}}";
    return call(kEndMapping, parameters);
  }

  int call_close() { return call(kStopNode, R"({"data":{}})"); }

 private:
  void on_status(const void* message, const char* topic) {
    const auto* status =
        static_cast<const std_msgs::msg::dds_::String_*>(message);
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      received_status_ = true;
      last_status_ = status->data();
    }
    std::cout << "[STATUS] " << topic << ": " << last_status_ << std::endl;
    status_condition_.notify_all();
  }

  int call(std::int32_t api_id, const std::string& parameters) {
    std::string response;
    const std::int32_t status_code = Call(api_id, parameters, response);
    std::cout << "[RPC] api_id=" << api_id << " status_code=" << status_code
              << " response=" << response << std::endl;
    if (status_code != 0) {
      std::cerr << "[ERROR] Unitree SLAM RPCが失敗しました" << std::endl;
      return 1;
    }
    std::cout << "[OK] Unitree SLAM RPCが成功しました" << std::endl;
    return 0;
  }

  unitree::robot::ChannelSubscriberPtr<std_msgs::msg::dds_::String_>
      info_subscriber_;
  unitree::robot::ChannelSubscriberPtr<std_msgs::msg::dds_::String_>
      key_info_subscriber_;
  std::mutex status_mutex_;
  std::condition_variable status_condition_;
  bool received_status_{false};
  std::string last_status_;
};

struct Arguments {
  std::string interface_name;
  std::string action;
  std::string map_path;
  int timeout_seconds{5};
};

Arguments parse_arguments(int argc, char** argv) {
  if (argc < 3) {
    throw std::invalid_argument(
        "usage: g1_onboard_lio <interface> <probe|start|stop|close> "
        "[--map PATH] [--timeout SEC]");
  }
  Arguments arguments{argv[1], argv[2], "", 5};
  for (int index = 3; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--map" && index + 1 < argc) {
      arguments.map_path = argv[++index];
    } else if (option == "--timeout" && index + 1 < argc) {
      arguments.timeout_seconds = std::stoi(argv[++index]);
    } else {
      throw std::invalid_argument("unknown or incomplete option: " + option);
    }
  }
  if (arguments.action == "stop" && arguments.map_path.empty()) {
    throw std::invalid_argument("stopには--map PATHが必要です");
  }
  return arguments;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Arguments arguments = parse_arguments(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(0,
                                                      arguments.interface_name);
    SlamClient client;
    client.initialize(static_cast<float>(arguments.timeout_seconds));

    if (arguments.action == "probe") {
      if (!client.wait_for_status(
              std::chrono::seconds(arguments.timeout_seconds))) {
        std::cerr << "[ERROR] slam_info/slam_key_infoを受信できませんでした"
                  << std::endl;
        return 1;
      }
      std::cout << "[OK] G1内蔵LIOサービスを確認しました" << std::endl;
      return 0;
    }
    if (arguments.action == "start") {
      return client.call_start();
    }
    if (arguments.action == "stop") {
      return client.call_stop(arguments.map_path);
    }
    if (arguments.action == "close") {
      return client.call_close();
    }
    throw std::invalid_argument("unknown action: " + arguments.action);
  } catch (const std::exception& error) {
    std::cerr << "[ERROR] " << error.what() << std::endl;
    return 2;
  }
}
