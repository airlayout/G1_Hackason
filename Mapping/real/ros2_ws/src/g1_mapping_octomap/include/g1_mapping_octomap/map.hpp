#pragma once

#include <octomap/octomap.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <vector>

namespace g1_mapping_octomap {

struct Config {
  double resolution{0.10};
  double voxel_size{0.05};
  double min_range{0.30};
  double max_range{3.0};
  double prob_hit{0.7};
  double prob_miss{0.4};
  double clamp_min{0.12};
  double clamp_max{0.97};
  bool keep_unknown{true};
  std::size_t max_points{2000000};
  unsigned target_scans{10};

  void validate() const {
    const std::array<double, 8> finite_values{
      resolution, voxel_size, min_range, max_range,
      prob_hit, prob_miss, clamp_min, clamp_max};
    for (double value : finite_values) {
      if (!std::isfinite(value)) throw std::invalid_argument("non-finite map parameter");
    }
    if (resolution <= 0 || voxel_size <= 0 || min_range < 0 || max_range <= min_range ||
        prob_hit <= 0.5 || prob_hit >= 1 || prob_miss <= 0 || prob_miss >= 0.5 ||
        clamp_min <= 0 || clamp_min >= 0.5 || clamp_max <= 0.5 || clamp_max >= 1 ||
        !max_points || !target_scans) {
      throw std::invalid_argument("invalid occupancy map parameters");
    }
  }
};

struct Point { float x, y, z, intensity; };
struct MapPoint {
  float x, y, z, intensity, density;
  std::uint32_t hit_count, scan_count;
  float occupancy;
};
static_assert(sizeof(MapPoint) == 32, "PointCloud2/PCD layout must be 32 bytes");

// The only map update implementation. Both executables use this class.
class OccupancyMap {
 public:
  explicit OccupancyMap(Config config) : config_(checked(config)), tree_(config_.resolution) {
    tree_.setProbHit(config_.prob_hit);
    tree_.setProbMiss(config_.prob_miss);
    tree_.setClampingThresMin(config_.clamp_min);
    tree_.setClampingThresMax(config_.clamp_max);
  }

  void insert(const std::vector<Point>& points, const std::array<double, 3>& origin) {
    for (double v : origin) {
      if (!std::isfinite(v)) throw std::invalid_argument("non-finite sensor origin");
    }
    octomap::Pointcloud scan;
    std::map<Key, Stats> observed;
    for (const auto& point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      const double range = std::sqrt(
        std::pow(point.x - origin[0], 2) + std::pow(point.y - origin[1], 2) +
        std::pow(point.z - origin[2], 2));
      if (range < config_.min_range) continue;
      // OctoMap keys have a finite coordinate range. Do not retain unrepresentable points.
      octomap::OcTreeKey tree_key;
      if (!tree_.coordToKeyChecked(point.x, point.y, point.z, tree_key)) continue;
      scan.push_back(point.x, point.y, point.z);
      observed[key(point)].add(point);
    }
    octomap::OcTreeKey origin_key;
    if (!tree_.coordToKeyChecked(origin[0], origin[1], origin[2], origin_key)) {
      throw std::invalid_argument("sensor origin outside OctoMap coordinate range");
    }
    if (scan.size() == 0) return;  // An empty scan is not evidence of free space.
    tree_.insertPointCloud(scan, octomap::point3d(origin[0], origin[1], origin[2]),
                           config_.max_range, false, false);
    ++scans_;
    if (candidates_.size() + observed.size() > config_.max_points) prune_free();
    for (const auto& [voxel, update] : observed) {
      auto found = candidates_.find(voxel);
      if (found == candidates_.end()) {
        if (candidates_.size() >= config_.max_points) { ++dropped_; continue; }
        found = candidates_.emplace(voxel, Stats{}).first;
      }
      auto& stats = found->second;
      stats.x += update.x; stats.y += update.y; stats.z += update.z;
      stats.intensity += update.intensity;
      stats.hits += update.hits;
      ++stats.scans;
    }
    // Clearing is part of each update, never a side effect of visualization/save.
    // Otherwise old points can reappear when the same cell becomes occupied later,
    // and live publishing cadence would change the offline reconstruction result.
    prune_free();
  }

  std::vector<MapPoint> snapshot() const {
    std::vector<MapPoint> result;
    result.reserve(candidates_.size());
    for (const auto& [voxel, stats] : candidates_) {
      (void)voxel;
      auto p = stats.point();
      const auto* node = tree_.search(p.x, p.y, p.z);
      if (node && !tree_.isNodeOccupied(node)) continue;
      if (!node && !config_.keep_unknown) continue;
      result.push_back({p.x, p.y, p.z, p.intensity,
        static_cast<float>(std::min(1.0, double(stats.scans) / config_.target_scans)),
        bounded(stats.hits), bounded(stats.scans),
        static_cast<float>(node ? node->getOccupancy() : 0.5)});
    }
    return result;
  }

  void save(const std::filesystem::path& path) {
    if (!scans_) throw std::runtime_error("no synchronized scans integrated; refusing to save");
    const auto points = snapshot();
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    const std::filesystem::path temporary = path.string() + ".tmp";
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    output.exceptions(std::ios::badbit | std::ios::failbit);
    output << "# .PCD v0.7\nVERSION 0.7\n"
      "FIELDS x y z intensity density hit_count scan_count occupancy\n"
      "SIZE 4 4 4 4 4 4 4 4\nTYPE F F F F F U U F\nCOUNT 1 1 1 1 1 1 1 1\n"
      "WIDTH " << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
      << points.size() << "\nDATA binary\n";
    output.write(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(MapPoint));
    output.close();
    auto tree_path = path; tree_path.replace_extension(".bt");
    const auto tree_temp = tree_path.string() + ".tmp";
    // writeBinary() would mutate the live tree into maximum-likelihood values.
    if (!tree_.writeBinaryConst(tree_temp)) throw std::runtime_error("OctoMap snapshot write failed");
    std::filesystem::rename(tree_temp, tree_path);
    auto config_path = path; config_path.replace_extension(".octomap.json");
    const auto config_temp = config_path.string() + ".tmp";
    std::ofstream metadata(config_temp);
    metadata.exceptions(std::ios::badbit | std::ios::failbit);
    metadata << std::setprecision(17) << "{\n  \"algorithm\": \"octomap\",\n  \"scans\": " << scans_
      << ",\n  \"candidate_limit_drops\": " << dropped_ << ",\n  \"parameters\": {"
      << "\"resolution\":" << config_.resolution << ",\"voxel_size\":" << config_.voxel_size
      << ",\"min_range\":" << config_.min_range << ",\"max_range\":" << config_.max_range
      << ",\"prob_hit\":" << config_.prob_hit << ",\"prob_miss\":" << config_.prob_miss
      << ",\"clamp_min\":" << config_.clamp_min << ",\"clamp_max\":" << config_.clamp_max
      << ",\"keep_unknown\":" << (config_.keep_unknown ? "true" : "false")
      << ",\"max_points\":" << config_.max_points << ",\"target_scans\":" << config_.target_scans
      << "}\n}\n";
    metadata.close();
    std::filesystem::rename(config_temp, config_path);
    std::filesystem::rename(temporary, path);
  }

  std::uint64_t scans() const { return scans_; }
  std::uint64_t dropped() const { return dropped_; }

 private:
  // Keep representatives inside their occupancy cell even for arbitrary output resolutions.
  using Key = std::array<std::int64_t, 6>;
  struct Stats {
    double x{}, y{}, z{}, intensity{};
    std::uint64_t hits{}, scans{};
    void add(const Point& p) {
      x += p.x; y += p.y; z += p.z;
      intensity += std::isfinite(p.intensity) ? p.intensity : 0;
      ++hits;
    }
    Point point() const {
      return {float(x / hits), float(y / hits), float(z / hits), float(intensity / hits)};
    }
  };
  static Config checked(Config config) { config.validate(); return config; }
  static std::uint32_t bounded(std::uint64_t n) {
    return static_cast<std::uint32_t>(std::min(n, std::uint64_t(UINT32_MAX)));
  }
  Key key(const Point& p) const {
    const auto cell = tree_.coordToKey(octomap::point3d(p.x, p.y, p.z));
    return {static_cast<std::int64_t>(std::floor(p.x / config_.voxel_size)),
            static_cast<std::int64_t>(std::floor(p.y / config_.voxel_size)),
            static_cast<std::int64_t>(std::floor(p.z / config_.voxel_size)),
            cell[0], cell[1], cell[2]};
  }
  void prune_free() {
    for (auto it = candidates_.begin(); it != candidates_.end();) {
      const auto p = it->second.point();
      const auto* node = tree_.search(p.x, p.y, p.z);
      if (node && !tree_.isNodeOccupied(node)) it = candidates_.erase(it);
      else ++it;
    }
  }
  Config config_;
  octomap::OcTree tree_;
  std::map<Key, Stats> candidates_;
  std::uint64_t scans_{}, dropped_{};
};

}  // namespace g1_mapping_octomap
