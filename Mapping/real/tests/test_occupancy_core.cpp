#include "g1_mapping_octomap/map.hpp"
#include <iostream>
#include <limits>
#include <chrono>

namespace gm = g1_mapping_octomap;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
bool has_x(const std::vector<gm::MapPoint>& points, float x) {
  for (const auto& p : points) if (std::abs(p.x - x) < 0.001) return true;
  return false;
}

int main() {
  try {
    const gm::Point ghost{11.05, 0.05, 0.55, 10};
    const gm::Point wall{12.55, 0.05, 0.55, 20};
    const std::array<double, 3> origin{10.05, 0.05, 0.55};
    gm::OccupancyMap map(gm::Config{});
    map.insert({ghost, wall}, origin);
    require(has_x(map.snapshot(), ghost.x), "first hit must initially be present");
    for (int i = 0; i < 100; ++i) map.insert({}, origin);
    require(has_x(map.snapshot(), ghost.x), "no-return/empty scans must not clear unseen space");
    map.insert({wall}, origin);
    require(has_x(map.snapshot(), ghost.x), "one miss should not erase one hit");
    for (int i = 0; i < 10; ++i) map.insert({wall}, origin);
    require(!has_x(map.snapshot(), ghost.x), "later free-space rays must erase the ghost");
    require(has_x(map.snapshot(), wall.x), "the static wall must survive");

    // Out-of-range endpoints are unknown, not occupied; conservative export keeps them.
    gm::Point far{15.05, 0.05, 0.55, 30};
    map.insert({far}, origin);
    require(has_x(map.snapshot(), far.x), "unknown distant structure must be preserved");
    gm::Config strict; strict.keep_unknown = false;
    gm::OccupancyMap known_only(strict);
    known_only.insert({far}, origin);
    require(known_only.snapshot().empty(), "strict export must omit unknown endpoints");

    // Full maps must reclaim cleared candidates instead of retaining the first ghost forever.
    gm::Config limited; limited.max_points = 1;
    gm::OccupancyMap small(limited);
    small.insert({ghost}, origin);
    for (int i = 0; i < 10; ++i) small.insert({wall}, origin);
    require(has_x(small.snapshot(), wall.x), "a cleared candidate must release capacity");
    require(!has_x(small.snapshot(), ghost.x), "capacity must not preserve stale noise");

    gm::Config invalid; invalid.max_range = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try { gm::OccupancyMap bad(invalid); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid sensor models must be rejected");

    // Saving a preview must not harden occupancy probabilities in the live map.
    const auto temporary = std::filesystem::temp_directory_path() /
      ("g1-occupancy-save-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    gm::OccupancyMap saved(gm::Config{});
    saved.insert({ghost, wall}, origin);
    saved.save(temporary / "map.pcd");
    for (int i = 0; i < 3; ++i) saved.insert({wall}, origin);
    require(!has_x(saved.snapshot(), ghost.x), "saving must preserve future clearing behavior");
    require(std::filesystem::is_regular_file(temporary / "map.octomap.json"), "configuration must be saved");
    std::filesystem::remove_all(temporary);

    // Display cadence must not affect clearing/re-observation or resurrect old samples.
    gm::OccupancyMap live(gm::Config{}), offline(gm::Config{});
    auto both = [&](const std::vector<gm::Point>& scan) {
      live.insert(scan, origin); live.snapshot();
      offline.insert(scan, origin);
    };
    both({ghost, wall});
    for (int i = 0; i < 10; ++i) both({wall});
    const gm::Point new_surface{11.01, 0.05, 0.55, 40};
    for (int i = 0; i < 10; ++i) both({new_surface, wall});
    require(live.snapshot().size() == offline.snapshot().size(), "publishing must not change the map");
    require(!has_x(offline.snapshot(), ghost.x), "cleared old samples must never reappear");
    std::cout << "[OK] OctoMap ghost removal, static/unknown preservation and capacity reclamation\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FAIL] " << e.what() << '\n'; return 1;
  }
}
