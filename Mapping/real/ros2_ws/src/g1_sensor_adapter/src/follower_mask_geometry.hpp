#pragma once

#include <cmath>

namespace g1_sensor_adapter {

enum class FollowerMaskShape {
  kBox,
  kRearSector,
};

struct FollowerMaskLimits {
  FollowerMaskShape shape{FollowerMaskShape::kBox};
  double min_x{-1.0};
  double max_x{1.0};
  double min_y{-1.0};
  double max_y{1.0};
  double min_z{-1.0};
  double max_z{1.0};
  double min_range{0.0};
  double max_range{2.0};
  double rear_half_angle_deg{90.0};
};

inline bool inside_follower_mask(float x, float y, float z,
                                 const FollowerMaskLimits& limits) {
  if (z < limits.min_z || z > limits.max_z) {
    return false;
  }
  if (limits.shape == FollowerMaskShape::kBox) {
    return x >= limits.min_x && x <= limits.max_x &&
           y >= limits.min_y && y <= limits.max_y;
  }

  const double range = std::sqrt(static_cast<double>(x) * x +
                                 static_cast<double>(y) * y +
                                 static_cast<double>(z) * z);
  if (range < limits.min_range || range > limits.max_range) {
    return false;
  }

  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  const double azimuth_deg = std::atan2(static_cast<double>(y),
                                        static_cast<double>(x)) *
                             kRadiansToDegrees;
  const double rear_offset_deg = 180.0 - std::abs(azimuth_deg);
  return rear_offset_deg <= limits.rear_half_angle_deg;
}

}  // namespace g1_sensor_adapter
