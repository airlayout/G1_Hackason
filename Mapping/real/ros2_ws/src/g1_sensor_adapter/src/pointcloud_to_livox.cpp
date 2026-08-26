#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

const PointField* find_field(const PointCloud2& cloud,
                             const std::vector<std::string>& names) {
  for (const auto& name : names) {
    const auto iterator =
        std::find_if(cloud.fields.begin(), cloud.fields.end(),
                     [&name](const PointField& field) { return field.name == name; });
    if (iterator != cloud.fields.end()) {
      return &*iterator;
    }
  }
  return nullptr;
}

template <typename T>
T read_value(const std::uint8_t* data) {
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

double read_number(const std::uint8_t* point, const PointField& field) {
  const std::uint8_t* data = point + field.offset;
  switch (field.datatype) {
    case PointField::INT8:
      return read_value<std::int8_t>(data);
    case PointField::UINT8:
      return read_value<std::uint8_t>(data);
    case PointField::INT16:
      return read_value<std::int16_t>(data);
    case PointField::UINT16:
      return read_value<std::uint16_t>(data);
    case PointField::INT32:
      return read_value<std::int32_t>(data);
    case PointField::UINT32:
      return read_value<std::uint32_t>(data);
    case PointField::FLOAT32:
      return read_value<float>(data);
    case PointField::FLOAT64:
      return read_value<double>(data);
    default:
      throw std::runtime_error("unsupported PointField datatype");
  }
}

std::uint8_t to_uint8(double value) {
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<std::uint8_t>(std::clamp(std::lround(value), 0L, 255L));
}

class PointCloudToLivox final : public rclcpp::Node {
 public:
  PointCloudToLivox() : Node("g1_pointcloud_to_livox") {
    const std::string input_points = declare_parameter<std::string>(
        "input_points_topic", "/utlidar/cloud_livox_mid360");
    const std::string input_imu = declare_parameter<std::string>(
        "input_imu_topic", "/utlidar/imu_livox_mid360");
    const std::string output_points = declare_parameter<std::string>(
        "output_points_topic", "/g1_mapping/livox");
    const std::string output_imu = declare_parameter<std::string>(
        "output_imu_topic", "/g1_mapping/imu");
    timestamp_mode_ = declare_parameter<std::string>("timestamp_mode", "auto");
    allow_inferred_time_ =
        declare_parameter<bool>("allow_inferred_time", true);
    scan_period_seconds_ =
        declare_parameter<double>("scan_period_seconds", 0.1);

    const auto input_qos = rclcpp::SensorDataQoS();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    points_publisher_ =
        create_publisher<livox_ros_driver2::msg::CustomMsg>(output_points,
                                                            output_qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_imu,
                                                             output_qos);
    points_subscription_ = create_subscription<PointCloud2>(
        input_points, input_qos,
        [this](PointCloud2::ConstSharedPtr message) { on_points(*message); });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        input_imu, input_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          imu_publisher_->publish(*message);
        });

    RCLCPP_INFO(get_logger(), "PointCloud2 %s -> Livox CustomMsg %s",
                input_points.c_str(), output_points.c_str());
    RCLCPP_INFO(get_logger(), "IMU %s -> %s", input_imu.c_str(),
                output_imu.c_str());
  }

 private:
  std::uint32_t offset_nanoseconds(double value, std::int64_t base_ns) const {
    double offset = 0.0;
    std::string mode = timestamp_mode_;
    if (mode == "auto") {
      const double base_seconds = static_cast<double>(base_ns) / 1.0e9;
      if (std::abs(value - base_seconds) < 3600.0) {
        mode = "absolute_seconds";
      } else if (std::abs(value) <= 1.0) {
        mode = "relative_seconds";
      } else {
        mode = "relative_nanoseconds";
      }
    }
    if (mode == "absolute_seconds") {
      offset = value * 1.0e9 - static_cast<double>(base_ns);
    } else if (mode == "relative_seconds") {
      offset = value * 1.0e9;
    } else if (mode == "relative_nanoseconds") {
      offset = value;
    } else {
      throw std::runtime_error("unknown timestamp_mode: " + mode);
    }
    if (!std::isfinite(offset)) {
      return 0;
    }
    offset = std::clamp(
        offset, 0.0,
        static_cast<double>(std::numeric_limits<std::uint32_t>::max()));
    return static_cast<std::uint32_t>(std::llround(offset));
  }

  void on_points(const PointCloud2& cloud) {
    try {
      convert_and_publish(cloud);
    } catch (const std::exception& error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "PointCloud2変換失敗: %s", error.what());
    }
  }

  void convert_and_publish(const PointCloud2& cloud) {
    if (cloud.is_bigendian) {
      throw std::runtime_error("big-endian PointCloud2 is not supported");
    }
    const PointField* x = find_field(cloud, {"x"});
    const PointField* y = find_field(cloud, {"y"});
    const PointField* z = find_field(cloud, {"z"});
    if (x == nullptr || y == nullptr || z == nullptr) {
      throw std::runtime_error("x/y/z fields are required");
    }
    const PointField* reflectivity =
        find_field(cloud, {"reflectivity", "intensity"});
    const PointField* tag = find_field(cloud, {"tag"});
    const PointField* line = find_field(cloud, {"line", "ring"});
    const PointField* timestamp =
        find_field(cloud, {"timestamp", "time", "t", "offset_time"});
    if (timestamp == nullptr && !allow_inferred_time_) {
      throw std::runtime_error(
          "point timestamp field is missing and inference is disabled");
    }

    const std::size_t point_count =
        static_cast<std::size_t>(cloud.width) * cloud.height;
    livox_ros_driver2::msg::CustomMsg output;
    output.header = cloud.header;
    output.timebase =
        static_cast<std::uint64_t>(rclcpp::Time(cloud.header.stamp).nanoseconds());
    output.lidar_id = 0;
    output.rsvd = {0, 0, 0};
    output.points.reserve(point_count);
    const std::int64_t base_ns =
        rclcpp::Time(cloud.header.stamp).nanoseconds();

    for (std::uint32_t row = 0; row < cloud.height; ++row) {
      for (std::uint32_t column = 0; column < cloud.width; ++column) {
        const std::size_t index =
            static_cast<std::size_t>(row) * cloud.width + column;
        const std::size_t byte_index =
            static_cast<std::size_t>(row) * cloud.row_step +
            static_cast<std::size_t>(column) * cloud.point_step;
        if (byte_index + cloud.point_step > cloud.data.size()) {
          throw std::runtime_error("PointCloud2 data is shorter than metadata");
        }
        const std::uint8_t* point = cloud.data.data() + byte_index;
        livox_ros_driver2::msg::CustomPoint converted;
        converted.x = static_cast<float>(read_number(point, *x));
        converted.y = static_cast<float>(read_number(point, *y));
        converted.z = static_cast<float>(read_number(point, *z));
        if (!std::isfinite(converted.x) || !std::isfinite(converted.y) ||
            !std::isfinite(converted.z)) {
          continue;
        }
        converted.reflectivity = reflectivity == nullptr
                                     ? 0
                                     : to_uint8(read_number(point, *reflectivity));
        converted.tag = tag == nullptr ? 0 : to_uint8(read_number(point, *tag));
        converted.line =
            line == nullptr ? 0 : to_uint8(read_number(point, *line));
        if (timestamp != nullptr && timestamp_mode_ != "infer") {
          converted.offset_time =
              offset_nanoseconds(read_number(point, *timestamp), base_ns);
        } else {
          const double fraction = point_count > 1
                                      ? static_cast<double>(index) /
                                            static_cast<double>(point_count - 1)
                                      : 0.0;
          converted.offset_time = static_cast<std::uint32_t>(std::llround(
              fraction * scan_period_seconds_ * 1.0e9));
          if (!reported_inferred_time_) {
            RCLCPP_WARN(get_logger(),
                        "各点時刻がないため点順序から時刻を推定します（縮退モード）");
            reported_inferred_time_ = true;
          }
        }
        output.points.push_back(converted);
      }
    }
    output.point_num = static_cast<std::uint32_t>(output.points.size());
    points_publisher_->publish(output);
  }

  std::string timestamp_mode_;
  bool allow_inferred_time_{true};
  bool reported_inferred_time_{false};
  double scan_period_seconds_{0.1};
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr points_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToLivox>());
  rclcpp::shutdown();
  return 0;
}
