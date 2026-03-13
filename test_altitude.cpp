#include "sensors.h"
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
  std::cout << "Testing altitude fusion implementation..." << std::endl;

  ComplementaryFilter filter(0.98f);
  IMUData imu_data;

  // Test 1: Level drone at sea level
  std::cout << "\nTest 1: Level drone at sea level" << std::endl;
  imu_data.accel_x = 0.0f;
  imu_data.accel_y = 0.0f;
  imu_data.accel_z = 9.81f;  // Gravity
  imu_data.gyro_x = 0.0f;
  imu_data.gyro_y = 0.0f;
  imu_data.gyro_z = 0.0f;
  imu_data.pressure = 101325.0f;  // Sea level

  for (int i = 0; i < 100; i++) {
    filter.update(imu_data, 0.01f);
  }

  float altitude = filter.getAltitude();
  Angles angles = filter.getAngles();

  std::cout << "  Roll: " << (angles.roll * 180.0f / M_PI) << "°" << std::endl;
  std::cout << "  Pitch: " << (angles.pitch * 180.0f / M_PI) << "°" << std::endl;
  std::cout << "  Altitude: " << altitude << " m" << std::endl;

  // At sea level, altitude should be close to 0
  assert(altitude < 5.0f && altitude > -5.0f);
  std::cout << "  PASS" << std::endl;

  // Test 2: Higher altitude (lower pressure)
  std::cout << "\nTest 2: Higher altitude (1000m equivalent)" << std::endl;
  filter.reset();

  // 1000m altitude = ~89875 Pa
  imu_data.pressure = 89875.0f;

  for (int i = 0; i < 100; i++) {
    filter.update(imu_data, 0.01f);
  }

  altitude = filter.getAltitude();
  std::cout << "  Altitude: " << altitude << " m" << std::endl;

  // Should be around 1000m
  assert(altitude > 900.0f && altitude < 1100.0f);
  std::cout << "  PASS" << std::endl;

  // Test 3: Vertical acceleration effect
  std::cout << "\nTest 3: Vertical acceleration integration" << std::endl;
  filter.reset();

  imu_data.pressure = 101325.0f;
  imu_data.accel_z = 10.81f;  // 1.0 m/s^2 upward acceleration

  for (int i = 0; i < 100; i++) {
    filter.update(imu_data, 0.01f);
  }

  altitude = filter.getAltitude();
  std::cout << "  Altitude after upward accel: " << altitude << " m" << std::endl;

  // Should show some vertical velocity component
  assert(altitude > 0.0f);
  std::cout << "  PASS" << std::endl;

  std::cout << "\nAll altitude fusion tests passed!" << std::endl;
  return 0;
}
