// Direct SQLite/CDR reading also works with interrupted bags lacking metadata.yaml.
#include "g1_mapping_octomap/scans.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sqlite3.h>
#include <iostream>
#include <set>

namespace gm = g1_mapping_octomap;
using Database = std::unique_ptr<sqlite3, decltype(&sqlite3_close)>;
using Statement = std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)>;

Database open_database(const std::filesystem::path& path) {
  sqlite3* raw = nullptr;
  const int status = sqlite3_open_v2(path.c_str(), &raw, SQLITE_OPEN_READONLY, nullptr);
  Database db(raw, sqlite3_close);
  if (status != SQLITE_OK) throw std::runtime_error("cannot open bag: " + path.string());
  return db;
}
Statement query(sqlite3* db, const char* sql) {
  sqlite3_stmt* raw = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &raw, nullptr) != SQLITE_OK) {
    throw std::runtime_error(sqlite3_errmsg(db));
  }
  return Statement(raw, sqlite3_finalize);
}
template<class Message>
std::shared_ptr<Message> deserialize(const void* data, std::size_t size) {
  rclcpp::SerializedMessage bytes(size);
  auto& message = bytes.get_rcl_serialized_message();
  std::memcpy(message.buffer, data, size); message.buffer_length = size;
  auto result = std::make_shared<Message>();
  rclcpp::Serialization<Message>().deserialize_message(&bytes, result.get());
  return result;
}

int main(int argc, char** argv) {
  try {
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; i += 2) {
      if (i + 1 >= argc) throw std::invalid_argument("options require values");
      if (!args.emplace(argv[i], argv[i + 1]).second) throw std::invalid_argument("duplicate option");
    }
    const std::set<std::string> allowed{"--bag-dir", "--output", "--cloud-topic", "--pose-topic",
      "--resolution", "--voxel-size", "--min-range", "--max-range", "--prob-hit", "--prob-miss",
      "--clamp-min", "--clamp-max", "--keep-unknown", "--max-points", "--target-scans"};
    for (const auto& [name, value] : args) {
      (void)value;
      if (!allowed.count(name)) throw std::invalid_argument("unknown option: " + name);
    }
    const auto bag_dir = args.at("--bag-dir");
    const auto output = args.at("--output");
    const auto cloud_topic = args.count("--cloud-topic") ? args.at("--cloud-topic") : "/g1_mapping/cloud_registered";
    const auto pose_topic = args.count("--pose-topic") ? args.at("--pose-topic") : "/g1_mapping/sensor_pose";
    gm::Config config;
    auto number = [&](const char* name, double& value) {
      if (args.count(name)) value = std::stod(args.at(name));
    };
    number("--resolution", config.resolution); number("--voxel-size", config.voxel_size);
    number("--min-range", config.min_range); number("--max-range", config.max_range);
    number("--prob-hit", config.prob_hit); number("--prob-miss", config.prob_miss);
    number("--clamp-min", config.clamp_min); number("--clamp-max", config.clamp_max);
    if (args.count("--keep-unknown")) {
      const auto value = args.at("--keep-unknown");
      if (value != "true" && value != "false") throw std::invalid_argument("invalid keep-unknown");
      config.keep_unknown = value == "true";
    }
    if (args.count("--max-points")) config.max_points = std::stoull(args.at("--max-points"));
    if (args.count("--target-scans")) config.target_scans = std::stoul(args.at("--target-scans"));
    gm::OccupancyMap map(config);
    gm::ScanPairs pairs([&](const gm::Cloud& cloud, const gm::Pose& pose) {
      const auto& p = pose.pose.position;
      map.insert(gm::read_points(cloud), {p.x, p.y, p.z});
    });
    std::vector<std::pair<std::int64_t, std::filesystem::path>> files;
    for (const auto& file : std::filesystem::directory_iterator(bag_dir)) {
      if (file.path().extension() != ".db3") continue;
      auto db = open_database(file.path());
      auto first = query(db.get(), "SELECT MIN(timestamp) FROM messages");
      if (sqlite3_step(first.get()) != SQLITE_ROW) throw std::runtime_error("cannot read bag timestamps");
      files.emplace_back(sqlite3_column_int64(first.get(), 0), file.path());
    }
    if (files.empty()) throw std::runtime_error("no db3 bags found");
    std::sort(files.begin(), files.end());
    std::set<std::string> found_topics;
    for (const auto& [first_stamp, path] : files) {
      (void)first_stamp;
      auto db = open_database(path);
      auto rows = query(db.get(),
        "SELECT t.name, t.type, m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
        "WHERE t.name=? OR t.name=? ORDER BY m.timestamp, m.id");
      sqlite3_bind_text(rows.get(), 1, cloud_topic.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(rows.get(), 2, pose_topic.c_str(), -1, SQLITE_TRANSIENT);
      int status;
      while ((status = sqlite3_step(rows.get())) == SQLITE_ROW) {
        const std::string topic(reinterpret_cast<const char*>(sqlite3_column_text(rows.get(), 0)));
        const std::string type(reinterpret_cast<const char*>(sqlite3_column_text(rows.get(), 1)));
        found_topics.insert(topic);
        const void* data = sqlite3_column_blob(rows.get(), 2);
        const auto size = sqlite3_column_bytes(rows.get(), 2);
        if (size <= 0) throw std::runtime_error("empty CDR message");
        if (topic == cloud_topic && type == "sensor_msgs/msg/PointCloud2") {
          pairs.cloud(deserialize<gm::Cloud>(data, size));
        } else if (topic == pose_topic && type == "geometry_msgs/msg/PoseStamped") {
          pairs.pose(deserialize<gm::Pose>(data, size));
        } else throw std::runtime_error("unexpected recorded message type: " + type);
      }
      if (status != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(db.get()));
    }
    if (!found_topics.count(cloud_topic) || !found_topics.count(pose_topic)) {
      throw std::runtime_error("bag needs registered scans AND /g1_mapping/sensor_pose; "
        "replay old raw LiDAR+IMU through the updated FAST-LIO2 pipeline first. "
        "Uncorrected accumulation is available explicitly with --method accumulate.");
    }
    map.save(output);
    std::cout << "[OK] OctoMap: " << map.scans() << " scans -> " << output << '\n'
      << "[PAIRS] matched=" << pairs.matched << " invalid=" << pairs.invalid
      << " evicted=" << pairs.dropped << " pending_clouds=" << pairs.pending_clouds()
      << " pending_poses=" << pairs.pending_poses() << " candidate_limit_drops=" << map.dropped() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] occupancy rebuild: " << e.what() << '\n';
    return 1;
  }
}
