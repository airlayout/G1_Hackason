// センサ座標系で直方体を切り抜く（または切り捨てる）フィルタ。
//
// **なぜ自前で書いたか。** 当初は `pcl_ros` の CropBox を使う設計だったが、
// **ROS 2 Humble の `pcl_ros` 2.4.5 はフィルタ群をビルドしていない**
// （`pcl_ros/CMakeLists.txt` の `add_library(pcl_ros_filters ...)` が
// タグ 2.4.5 でも humble ブランチでも `#` でコメントアウトされている。
// 2026-09-04 に実機イメージ内で `ros2 pkg executables pcl_ros` が
// `pcd_to_pointcloud` しか返さないことを確認した）。
//
// 代替として Autoware の `autoware_pointcloud_preprocessor` に CropBox があるが、
// autoware_utils / autoware_point_types など依存が大きい。処理自体は
// **1点あたり6回の比較**なので、既に持っているパッケージに置いた。
//
// **パラメータ名と topic 名は `pcl_ros::CropBox` に合わせてある**ので、
// 将来 pcl_ros のフィルタが Humble で使えるようになったら、launch の
// package/executable を差し替えるだけで入れ替わる。
//
// 点は `point_step` 単位でまるごとコピーするので、x/y/z 以外のフィールド
// （intensity / ring / time）はそのまま保たれる。FAST-LIO2 の de-skew は
// per-point の `time` を使うため、これが落ちると歪みが直らない。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "follower_mask_geometry.hpp"

namespace {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using g1_sensor_adapter::FollowerMaskLimits;
using g1_sensor_adapter::FollowerMaskShape;
using g1_sensor_adapter::inside_follower_mask;

// x/y/z の offset を引く。float32 以外は扱わない（Livox も内蔵SLAMも float32）。
struct XyzLayout {
  std::uint32_t x_offset;
  std::uint32_t y_offset;
  std::uint32_t z_offset;
};

XyzLayout resolve_xyz(const PointCloud2& cloud) {
  XyzLayout layout{};
  bool found_x = false, found_y = false, found_z = false;
  for (const auto& field : cloud.fields) {
    if (field.datatype != PointField::FLOAT32) {
      continue;
    }
    if (field.name == "x") { layout.x_offset = field.offset; found_x = true; }
    else if (field.name == "y") { layout.y_offset = field.offset; found_y = true; }
    else if (field.name == "z") { layout.z_offset = field.offset; found_z = true; }
  }
  if (!found_x || !found_y || !found_z) {
    throw std::runtime_error("x/y/z (float32) が点群にありません");
  }
  return layout;
}

float read_float(const std::uint8_t* point, std::uint32_t offset) {
  float value = 0.0F;
  std::memcpy(&value, point + offset, sizeof(float));
  return value;
}

class CropBox final : public rclcpp::Node {
 public:
  CropBox() : rclcpp::Node("crop_box") {
    const std::string shape = declare_parameter<std::string>("shape", "box");
    if (shape == "box") {
      limits_.shape = FollowerMaskShape::kBox;
    } else if (shape == "rear_sector") {
      limits_.shape = FollowerMaskShape::kRearSector;
    } else {
      throw std::runtime_error("shape は box または rear_sector で指定してください");
    }
    limits_.min_x = declare_parameter<double>("min_x", -1.0);
    limits_.max_x = declare_parameter<double>("max_x", 1.0);
    limits_.min_y = declare_parameter<double>("min_y", -1.0);
    limits_.max_y = declare_parameter<double>("max_y", 1.0);
    limits_.min_z = declare_parameter<double>("min_z", -1.0);
    limits_.max_z = declare_parameter<double>("max_z", 1.0);
    limits_.min_range = declare_parameter<double>("min_range", 0.45);
    limits_.max_range = declare_parameter<double>("max_range", 1.8);
    limits_.rear_half_angle_deg =
        declare_parameter<double>("rear_half_angle_deg", 90.0);
    // true なら箱の中を捨てる。false なら箱の中だけ残す（pcl_ros と同じ意味）
    negative_ = declare_parameter<bool>("negative", false);

    if (limits_.min_x > limits_.max_x || limits_.min_y > limits_.max_y ||
        limits_.min_z > limits_.max_z ||
        limits_.min_range > limits_.max_range ||
        limits_.rear_half_angle_deg < 0.0 ||
        limits_.rear_half_angle_deg > 180.0) {
      throw std::runtime_error("min が max を超えています。箱の指定を確認してください");
    }

    // QoS は送り側（bag play / DDS ブリッジ）に合わせて best_effort も拾えるようにする。
    // 深さを大きめに取るのは、再生の瞬間的な詰まりで落とさないため。
    const auto qos = rclcpp::SensorDataQoS().keep_last(20);
    publisher_ = create_publisher<PointCloud2>("output", qos);
    subscription_ = create_subscription<PointCloud2>(
        "input", qos,
        [this](PointCloud2::ConstSharedPtr message) { on_points(*message); });

    if (limits_.shape == FollowerMaskShape::kBox) {
      RCLCPP_INFO(
          get_logger(),
          "FollowerMask box x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f] negative=%s",
          limits_.min_x, limits_.max_x, limits_.min_y, limits_.max_y,
          limits_.min_z, limits_.max_z, negative_ ? "true" : "false");
    } else {
      RCLCPP_INFO(
          get_logger(),
          "FollowerMask rear_sector range[%.2f, %.2f] angle=+/-%.1fdeg z[%.2f, %.2f] negative=%s",
          limits_.min_range, limits_.max_range, limits_.rear_half_angle_deg,
          limits_.min_z, limits_.max_z, negative_ ? "true" : "false");
    }
  }

 private:
  bool inside(float x, float y, float z) const {
    return inside_follower_mask(x, y, z, limits_);
  }

  void on_points(const PointCloud2& cloud) {
    XyzLayout layout;
    try {
      layout = resolve_xyz(cloud);
    } catch (const std::exception& error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
      return;
    }
    const std::size_t step = cloud.point_step;
    if (step == 0 || cloud.data.size() < step) {
      return;
    }
    const std::size_t count = cloud.data.size() / step;

    PointCloud2 output;
    output.header = cloud.header;
    output.fields = cloud.fields;
    output.is_bigendian = cloud.is_bigendian;
    output.point_step = cloud.point_step;
    output.is_dense = cloud.is_dense;
    output.data.reserve(cloud.data.size());

    std::size_t kept = 0;
    for (std::size_t index = 0; index < count; ++index) {
      const std::uint8_t* point = cloud.data.data() + index * step;
      const bool in_box = inside(read_float(point, layout.x_offset),
                                 read_float(point, layout.y_offset),
                                 read_float(point, layout.z_offset));
      if (in_box == negative_) {
        continue;  // negative=true なら箱の中を捨てる
      }
      // フィールド構成を保つため、点をまるごと写す
      output.data.insert(output.data.end(), point, point + step);
      ++kept;
    }

    // 非organized（height=1）として出す。Livox は元から height=1。
    output.height = 1;
    output.width = static_cast<std::uint32_t>(kept);
    output.row_step = static_cast<std::uint32_t>(kept * step);

    dropped_total_ += count - kept;
    input_total_ += count;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "除去 %.2f%%（累計 %zu / %zu 点）",
        input_total_ ? 100.0 * static_cast<double>(dropped_total_) /
                           static_cast<double>(input_total_) : 0.0,
        dropped_total_, input_total_);

    publisher_->publish(output);
  }

  FollowerMaskLimits limits_;
  bool negative_{false};
  std::size_t dropped_total_{0};
  std::size_t input_total_{0};
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropBox>());
  rclcpp::shutdown();
  return 0;
}
