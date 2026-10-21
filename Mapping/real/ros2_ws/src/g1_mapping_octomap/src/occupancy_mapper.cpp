#include "g1_mapping_octomap/scans.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <iostream>

namespace gm = g1_mapping_octomap;

class Mapper : public rclcpp::Node {
 public:
  Mapper() : Node("g1_occupancy_mapper") {
    gm::Config config;
    config.resolution = declare_parameter("resolution", config.resolution);
    config.voxel_size = declare_parameter("voxel_size", config.voxel_size);
    config.min_range = declare_parameter("min_range", config.min_range);
    config.max_range = declare_parameter("max_range", config.max_range);
    config.prob_hit = declare_parameter("prob_hit", config.prob_hit);
    config.prob_miss = declare_parameter("prob_miss", config.prob_miss);
    config.clamp_min = declare_parameter("clamp_min", config.clamp_min);
    config.clamp_max = declare_parameter("clamp_max", config.clamp_max);
    config.keep_unknown = declare_parameter("keep_unknown", config.keep_unknown);
    const int max_points = declare_parameter("max_points", int(config.max_points));
    const int target_scans = declare_parameter("target_scans", int(config.target_scans));
    if (max_points <= 0 || target_scans <= 0) throw std::invalid_argument("invalid map limits");
    config.max_points = max_points; config.target_scans = target_scans;
    map_ = std::make_unique<gm::OccupancyMap>(config);
    output_ = declare_parameter("output_path", std::string("/runs/unknown/map/map_raw.pcd"));
    const auto frame = declare_parameter("global_frame", std::string("map"));
    const auto cloud_topic = declare_parameter("cloud_topic", std::string("/g1_mapping/cloud_registered"));
    const auto pose_topic = declare_parameter("sensor_pose_topic", std::string("/g1_mapping/sensor_pose"));
    const double hz = declare_parameter("publish_hz", 1.0);
    if (!std::isfinite(hz) || hz <= 0) throw std::invalid_argument("invalid publish_hz");
    frame_ = frame;
    pairs_ = std::make_unique<gm::ScanPairs>([this](const gm::Cloud& cloud, const gm::Pose& pose) {
      const auto& p = pose.pose.position;
      map_->insert(gm::read_points(cloud), {p.x, p.y, p.z});
      latest_ = cloud.header; dirty_ = true;
    }, frame);
    publisher_ = create_publisher<gm::Cloud>("/g1_mapping/map", rclcpp::QoS(1).transient_local().reliable());
    cloud_sub_ = create_subscription<gm::Cloud>(cloud_topic, rclcpp::SensorDataQoS().keep_last(128),
      [this](gm::Cloud::ConstSharedPtr msg) {
        try { pairs_->cloud(std::move(msg)); }
        catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "scan rejected: %s", e.what()); }
      });
    pose_sub_ = create_subscription<gm::Pose>(pose_topic, rclcpp::SensorDataQoS().keep_last(128),
      [this](gm::Pose::ConstSharedPtr msg) {
        try { pairs_->pose(std::move(msg)); }
        catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "pose rejected: %s", e.what()); }
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this] { publish(); });
    save_service_ = create_service<std_srvs::srv::Trigger>("/g1_mapping/save_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        try { map_->save(output_); response->success = true; response->message = output_; }
        catch (const std::exception& e) { response->success = false; response->message = e.what(); }
      });
    RCLCPP_INFO(get_logger(), "OctoMap: exact scan/sensor-pose pairs -> /g1_mapping/map -> %s", output_.c_str());
  }
  void save_on_exit() {
    if (map_->scans()) map_->save(output_);
  }
 private:
  void publish() {
    if (!dirty_) return;
    const auto points = map_->snapshot();
    gm::Cloud cloud;
    cloud.header = latest_; cloud.header.frame_id = frame_;
    cloud.height = 1; cloud.width = points.size(); cloud.is_dense = true;
    const std::array<std::string, 8> names{
      "x", "y", "z", "intensity", "density", "hit_count", "scan_count", "occupancy"};
    for (unsigned i = 0; i < names.size(); ++i) {
      sensor_msgs::msg::PointField field;
      field.name = names[i]; field.offset = i * 4; field.count = 1;
      field.datatype = (i == 5 || i == 6) ? field.UINT32 : field.FLOAT32;
      cloud.fields.push_back(field);
    }
    cloud.point_step = sizeof(gm::MapPoint); cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize(cloud.row_step);
    if (!points.empty()) std::memcpy(cloud.data.data(), points.data(), cloud.row_step);
    publisher_->publish(cloud); dirty_ = false;
    RCLCPP_WARN_EXPRESSION(get_logger(), pairs_->invalid || pairs_->dropped || map_->dropped(),
      "map diagnostics: invalid_pairs=%lu unmatched_evicted=%lu candidate_limit_drops=%lu",
      pairs_->invalid, pairs_->dropped, map_->dropped());
  }
  std::unique_ptr<gm::OccupancyMap> map_;
  std::unique_ptr<gm::ScanPairs> pairs_;
  std::string output_, frame_;
  std_msgs::msg::Header latest_;
  bool dirty_{false};
  rclcpp::Publisher<gm::Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<gm::Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<gm::Pose>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto mapper = std::make_shared<Mapper>();
    rclcpp::spin(mapper);
    mapper->save_on_exit();
  } catch (const std::exception& e) {
    std::cerr << "occupancy mapper: " << e.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
