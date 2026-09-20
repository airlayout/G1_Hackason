#include "follower_mask_geometry.hpp"

#include <cassert>

using g1_sensor_adapter::FollowerMaskLimits;
using g1_sensor_adapter::FollowerMaskShape;
using g1_sensor_adapter::inside_follower_mask;

int main() {
  FollowerMaskLimits box;
  box.shape = FollowerMaskShape::kBox;
  box.min_x = -1.9;
  box.max_x = -0.5;
  box.min_y = -0.8;
  box.max_y = 0.8;
  box.min_z = -0.2;
  box.max_z = 1.2;
  assert(inside_follower_mask(-1.0F, 0.0F, 0.5F, box));
  assert(!inside_follower_mask(-0.4F, 0.0F, 0.5F, box));

  FollowerMaskLimits sector;
  sector.shape = FollowerMaskShape::kRearSector;
  sector.min_range = 0.45;
  sector.max_range = 1.8;
  sector.rear_half_angle_deg = 90.0;
  sector.min_z = -0.2;
  sector.max_z = 1.2;

  assert(inside_follower_mask(-1.0F, 0.0F, 0.5F, sector));
  assert(inside_follower_mask(0.0F, 1.0F, 0.5F, sector));
  assert(inside_follower_mask(0.0F, -1.0F, 0.5F, sector));
  assert(!inside_follower_mask(1.0F, 0.0F, 0.5F, sector));
  assert(!inside_follower_mask(-2.0F, 0.0F, 0.5F, sector));
  assert(!inside_follower_mask(-1.0F, 0.0F, 1.3F, sector));

  // 180度なら全方位。ただし距離・高さの制限は維持する。
  sector.rear_half_angle_deg = 180.0;
  assert(inside_follower_mask(1.0F, 0.0F, 0.5F, sector));
  assert(!inside_follower_mask(2.0F, 0.0F, 0.5F, sector));
  return 0;
}
