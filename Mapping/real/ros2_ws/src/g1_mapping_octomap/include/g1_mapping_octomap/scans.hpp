#pragma once

#include "g1_mapping_octomap/map.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

namespace g1_mapping_octomap {
using Cloud = sensor_msgs::msg::PointCloud2;
using Pose = geometry_msgs::msg::PoseStamped;

inline std::int64_t stamp(const std_msgs::msg::Header& header) {
  return std::int64_t(header.stamp.sec) * 1000000000 + header.stamp.nanosec;
}

inline std::vector<Point> read_points(const Cloud& cloud) {
  if (cloud.is_bigendian) throw std::runtime_error("big-endian scans are unsupported");
  std::array<int, 4> offsets{-1, -1, -1, -1};
  const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
  for (const auto& field : cloud.fields) {
    for (unsigned i = 0; i < names.size(); ++i) {
      if (field.name != names[i]) continue;
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1 ||
          std::uint64_t(field.offset) + 4 > cloud.point_step) {
        throw std::runtime_error("scan fields must be scalar float32 inside point_step");
      }
      offsets[i] = static_cast<int>(field.offset);
    }
  }
  if (offsets[0] < 0 || offsets[1] < 0 || offsets[2] < 0 ||
      cloud.row_step < std::uint64_t(cloud.width) * cloud.point_step ||
      std::uint64_t(cloud.row_step) * cloud.height > cloud.data.size()) {
    throw std::runtime_error("invalid PointCloud2 layout");
  }
  std::vector<Point> points;
  points.reserve(std::size_t(cloud.width) * cloud.height);
  for (unsigned row = 0; row < cloud.height; ++row) {
    for (unsigned col = 0; col < cloud.width; ++col) {
      const auto* data = cloud.data.data() + std::size_t(row) * cloud.row_step +
                         std::size_t(col) * cloud.point_step;
      std::array<float, 4> value{};
      for (unsigned i = 0; i < offsets.size(); ++i) {
        if (offsets[i] >= 0) std::memcpy(&value[i], data + offsets[i], sizeof(float));
      }
      points.push_back({value[0], value[1], value[2], value[3]});
    }
  }
  return points;
}

// Exact scan-end stamps, never "latest odom" or a guessed LiDAR/IMU lever arm.
class ScanPairs {
 public:
  using Callback = std::function<void(const Cloud&, const Pose&)>;
  explicit ScanPairs(Callback callback, std::string frame = "map", std::size_t limit = 128)
    : callback_(std::move(callback)), frame_(std::move(frame)), limit_(limit) {}
  void cloud(std::shared_ptr<const Cloud> message) {
    const auto time = stamp(message->header);
    clouds_[time] = std::move(message); process();
  }
  void pose(std::shared_ptr<const Pose> message) {
    const auto time = stamp(message->header);
    poses_[time] = std::move(message); process();
  }
  std::uint64_t matched{}, dropped{}, invalid{};
  std::int64_t last_stamp{};
  std::size_t pending_clouds() const { return clouds_.size(); }
  std::size_t pending_poses() const { return poses_.size(); }
 private:
  void process() {
    for (auto it = clouds_.begin(); it != clouds_.end();) {
      auto pose = poses_.find(it->first);
      if (pose == poses_.end()) { ++it; continue; }
      const auto time = it->first;
      const auto cloud_message = it->second;
      const auto pose_message = pose->second;
      poses_.erase(pose);
      it = clouds_.erase(it);
      if (time <= last_stamp || time <= 0 ||
          cloud_message->header.frame_id != frame_ || pose_message->header.frame_id != frame_) {
        ++invalid;
      } else {
        try { callback_(*cloud_message, *pose_message); }
        catch (...) { ++invalid; throw; }
        last_stamp = time;
        ++matched;
      }
    }
    while (clouds_.size() > limit_) { clouds_.erase(clouds_.begin()); ++dropped; }
    while (poses_.size() > limit_) { poses_.erase(poses_.begin()); ++dropped; }
  }
  Callback callback_;
  std::string frame_;
  std::size_t limit_;
  std::map<std::int64_t, std::shared_ptr<const Cloud>> clouds_;
  std::map<std::int64_t, std::shared_ptr<const Pose>> poses_;
};
}  // namespace g1_mapping_octomap
