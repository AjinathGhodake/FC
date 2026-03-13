#include "sensors.h"

ComplementaryFilter::ComplementaryFilter(float alpha)
    : alpha(alpha) {
  reset();
}

void ComplementaryFilter::update(const IMUData& imu_data, float dt) {
  // Calculate accelerometer-based angles
  Angles accel_angles = calculateAccelAngles(imu_data);

  // Complementary filter formula:
  // angle = alpha * (gyro_integrated) + (1 - alpha) * accel_angle
  // Integrate gyro to get angle change over time

  float gyro_roll = imu_data.gyro_x * dt;
  float gyro_pitch = imu_data.gyro_y * dt;

  // Update angles using complementary filter
  // alpha = 0.98 means: 98% from gyro integration, 2% from accel
  angles.roll = alpha * (angles.roll + gyro_roll)
                + (1.0f - alpha) * accel_angles.roll;

  angles.pitch = alpha * (angles.pitch + gyro_pitch)
                 + (1.0f - alpha) * accel_angles.pitch;

  // For yaw, we only integrate gyro (no accelerometer information)
  angles.yaw += imu_data.gyro_z * dt;
  // Wrap yaw to [-π, π] to prevent overflow
  while (angles.yaw > M_PI) angles.yaw -= 2.0f * M_PI;
  while (angles.yaw < -M_PI) angles.yaw += 2.0f * M_PI;
}

Angles ComplementaryFilter::getAngles() const {
  return angles;
}

void ComplementaryFilter::reset() {
  angles.roll = 0.0f;
  angles.pitch = 0.0f;
  angles.yaw = 0.0f;
}

Angles ComplementaryFilter::calculateAccelAngles(const IMUData& imu_data) const {
  Angles accel_angles;

  // Calculate roll from accelerometer
  // tan(roll) = accel_y / accel_z
  accel_angles.roll = std::atan2(imu_data.accel_y, imu_data.accel_z);

  // Calculate pitch from accelerometer
  // sin(pitch) = -accel_x / g
  // Using atan2 for robustness: tan(pitch) = -accel_x / sqrt(accel_y^2 + accel_z^2)
  float az_magnitude = std::sqrt(imu_data.accel_y * imu_data.accel_y +
                                 imu_data.accel_z * imu_data.accel_z);
  if (az_magnitude < 0.1f) {  // Near-zero check for safety
    accel_angles.pitch = 0.0f;
  } else {
    accel_angles.pitch = std::atan2(-imu_data.accel_x, az_magnitude);
  }

  // Yaw cannot be calculated from accelerometer alone
  accel_angles.yaw = 0.0f;

  return accel_angles;
}
