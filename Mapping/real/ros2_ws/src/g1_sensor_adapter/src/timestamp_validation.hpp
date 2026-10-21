#pragma once
#include <cstdint>

namespace g1_sensor_adapter {
inline bool valid_scan_stamp(std::int64_t nanoseconds, bool simulation_time) {
  // Simulation clocks start near zero. Real sensors must carry a plausible epoch.
  constexpr std::int64_t minimum_epoch = 1000000000LL * 1000000000LL;
  return simulation_time ? nanoseconds > 0 : nanoseconds >= minimum_epoch;
}
}  // namespace g1_sensor_adapter
