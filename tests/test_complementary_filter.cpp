#include "../sensors.h"
#include <cstdio>
#include <cassert>
#include <cmath>

// Tolerance for floating point comparisons
const float TOLERANCE = 0.01f;  // 0.01 radians ≈ 0.57 degrees

// Test helper: check if two values are approximately equal
bool almostEqual(float a, float b, float tol = TOLERANCE) {
  return std::abs(a - b) < tol;
}

// Test 1: Stationary sensor (no motion)
// Expected: roll and pitch should remain at ~0 degrees
void test_stationary() {
  printf("Test 1: Stationary sensor...\n");

  ComplementaryFilter filter(0.98f);

  IMUData imu;
  imu.accel_x = 0.0f;
  imu.accel_y = 0.0f;
  imu.accel_z = 9.81f;  // Only gravity
  imu.gyro_x = 0.0f;
  imu.gyro_y = 0.0f;
  imu.gyro_z = 0.0f;

  // Simulate 10 updates at 100 Hz (0.01 second intervals)
  for (int i = 0; i < 10; i++) {
    filter.update(imu, 0.01f);
  }

  Angles angles = filter.getAngles();

  printf("  Roll: %.4f rad (%.2f deg)\n", angles.roll, angles.roll * 180.0f / M_PI);
  printf("  Pitch: %.4f rad (%.2f deg)\n", angles.pitch, angles.pitch * 180.0f / M_PI);
  printf("  Yaw: %.4f rad (%.2f deg)\n", angles.yaw, angles.yaw * 180.0f / M_PI);

  assert(almostEqual(angles.roll, 0.0f, 0.05f));
  assert(almostEqual(angles.pitch, 0.0f, 0.05f));

  printf("  ✓ PASSED\n\n");
}

// Test 2: 45-degree tilt around Y-axis (pitch)
// When tilted 45 degrees on pitch axis (nose up):
//   accel_x ≈ -g * sin(45°) ≈ -6.94 m/s^2
//   accel_z ≈ g * cos(45°) ≈ 6.94 m/s^2
// Expected: pitch should converge toward ~45 degrees
void test_45_degree_tilt() {
  printf("Test 2: 45-degree tilt (pitch axis)...\n");

  ComplementaryFilter filter(0.98f);

  float angle_45_deg = M_PI / 4.0f;  // 45 degrees in radians
  float g = 9.81f;

  IMUData imu;
  imu.accel_x = -g * std::sin(angle_45_deg);  // ~-6.94 m/s^2
  imu.accel_y = 0.0f;
  imu.accel_z = g * std::cos(angle_45_deg);   // ~6.94 m/s^2
  imu.gyro_x = 0.0f;
  imu.gyro_y = 0.0f;
  imu.gyro_z = 0.0f;

  // Simulate 500 updates to let filter converge
  // At 100 Hz (0.01s per update), this is 5 seconds of simulated time
  // With alpha=0.98, convergence takes time but is stable
  for (int i = 0; i < 500; i++) {
    filter.update(imu, 0.01f);
  }

  Angles angles = filter.getAngles();

  printf("  Roll: %.4f rad (%.2f deg)\n", angles.roll, angles.roll * 180.0f / M_PI);
  printf("  Pitch: %.4f rad (%.2f deg)\n", angles.pitch, angles.pitch * 180.0f / M_PI);
  printf("  Yaw: %.4f rad (%.2f deg)\n", angles.yaw, angles.yaw * 180.0f / M_PI);

  float expected_pitch = angle_45_deg;
  printf("  Expected pitch: %.4f rad (%.2f deg)\n", expected_pitch, expected_pitch * 180.0f / M_PI);

  assert(almostEqual(angles.roll, 0.0f, 0.05f));
  assert(almostEqual(angles.pitch, expected_pitch, 0.05f));

  printf("  ✓ PASSED\n\n");
}

int main() {
  printf("=== Complementary Filter Tests ===\n\n");

  test_stationary();
  test_45_degree_tilt();

  printf("=== All tests passed! ===\n");
  return 0;
}
