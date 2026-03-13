#ifndef SENSORS_H
#define SENSORS_H

#include <cmath>

// Struct to hold IMU readings (accelerometer and gyroscope)
struct IMUData {
  float accel_x;  // m/s^2
  float accel_y;  // m/s^2
  float accel_z;  // m/s^2
  float gyro_x;   // rad/s
  float gyro_y;   // rad/s
  float gyro_z;   // rad/s
};

// Struct to hold orientation angles
struct Angles {
  float roll;     // radians
  float pitch;    // radians
  float yaw;      // radians
};

// Complementary Filter class for sensor fusion
class ComplementaryFilter {
public:
  ComplementaryFilter(float alpha = 0.98f);

  // Update filter with new IMU data and time delta
  void update(const IMUData& imu_data, float dt);

  // Get current angle estimates
  Angles getAngles() const;

  // Reset filter state
  void reset();

private:
  float alpha;        // Weight for gyro integration (0.98 typical)
  Angles angles;      // Current angle estimates

  // Helper function to calculate accel-based angles
  Angles calculateAccelAngles(const IMUData& imu_data) const;
};

#endif // SENSORS_H
