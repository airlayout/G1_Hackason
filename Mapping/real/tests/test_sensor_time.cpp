#include "timestamp_validation.hpp"
#include <cassert>

int main() {
  using g1_sensor_adapter::valid_scan_stamp;
  assert(!valid_scan_stamp(0, true));
  assert(!valid_scan_stamp(0, false));
  assert(!valid_scan_stamp(-1, true));
  assert(valid_scan_stamp(5000000, true));
  assert(!valid_scan_stamp(5000000, false));
  assert(valid_scan_stamp(1792500000000000000LL, false));
  return 0;
}
