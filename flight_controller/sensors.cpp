#include <Arduino.h>
#include "sensors.h"
#include "config.h"
#include "gps.h"

ComplementaryFilter::ComplementaryFilter(float alpha)
    : alpha(alpha), madgwick(MADGWICK_BETA, MADGWICK_SAMPLE_RATE) {
  reset();
}

void ComplementaryFilter::update(const IMUData& imu_data, const GPSData* gps_data, float dt) {
#ifdef USE_MADGWICK
  // ========== Madgwick Filter (Gyro + Accel + Mag) ==========
  // Quaternion-based fusion for drift-free roll, pitch, and yaw
  madgwick.update(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                  imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                  imu_data.mag_x,   imu_data.mag_y,   imu_data.mag_z);

  angles.roll = madgwick.getRoll();
  angles.pitch = madgwick.getPitch();
  angles.yaw = madgwick.getYaw();
#else
  // ========== Complementary Filter (Gyro + Accel) ==========
  // Fast gyro with accel drift correction
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

  // For yaw, integrate gyro and correct with magnetometer (compass)
  // 98% gyro (fast, responsive) + 2% compass (drift-free anchor)
  angles.yaw += imu_data.gyro_z * dt;

  // Calculate compass-based yaw from magnetometer X and Y
  float compass_yaw = atan2f(imu_data.mag_y, imu_data.mag_x);

  // Calculate shortest angular distance between gyro yaw and compass yaw
  float yaw_diff = compass_yaw - angles.yaw;
  while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

  // Apply 2% compass correction (slow, stable anchor to prevent drift)
  angles.yaw += 0.02f * yaw_diff;

  // Wrap yaw to [-π, π] to prevent overflow
  while (angles.yaw > M_PI) angles.yaw -= 2.0f * M_PI;
  while (angles.yaw < -M_PI) angles.yaw += 2.0f * M_PI;
#endif

  // ========== Altitude Fusion ==========
  // Transform vertical acceleration to world frame using current attitude
  // accel_z is typically pointing up when level (compensated by gravity)
  // World frame Z (up): derived from body frame accel_z rotated by roll/pitch
  float cos_roll = cosf(angles.roll);
  float sin_roll = sinf(angles.roll);
  float cos_pitch = cosf(angles.pitch);
  float sin_pitch = sinf(angles.pitch);

  // Rotation matrix for body-to-world (simplified for Z-axis):
  // Z_world = accel_z * cos(roll) * cos(pitch) - gravity_compensation
  float accel_z_world = imu_data.accel_z * cos_roll * cos_pitch - 9.81f;

  // Integrate vertical acceleration to get vertical velocity (m/s)
  altitude_velocity += accel_z_world * dt;

  // Get barometer-based altitude from pressure
  float baro_altitude = pressureToAltitude(imu_data.pressure);

  // Complementary filter for altitude (barometer + accel):
  // 90% trust barometer (absolute), 10% trust accel velocity
  altitude = ALT_BARO_WEIGHT * baro_altitude +
             ALT_ACCEL_WEIGHT * altitude_velocity;

  // GPS altitude correction (faster convergence for calibration)
  // If GPS has a valid fix, blend GPS altitude to calibrate barometer
  // This corrects sea-level pressure errors and long-term drift
  if (gps_data != nullptr && gps_data->fix_valid) {
    // Use stronger correction (5%) so barometer converges to GPS in ~20 seconds
    // After convergence, GPS stays as the altitude anchor
    altitude = 0.95f * altitude + 0.05f * gps_data->altitude_gps;
  }
}

Angles ComplementaryFilter::getAngles() const {
  return angles;
}

void ComplementaryFilter::reset() {
  angles.roll = 0.0f;
  angles.pitch = 0.0f;
  angles.yaw = 0.0f;
  altitude = 0.0f;
  altitude_velocity = 0.0f;
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

float ComplementaryFilter::getAltitude() const {
  return altitude;
}

float ComplementaryFilter::pressureToAltitude(float pressure_pa) const {
  // Barometric altitude formula: h = (P0 / P)^(1/5.255) * T0 - T0
  // Simplified: h = 44330 * (1.0 - (P/P0)^(1/5.255))
  // where P0 = sea level pressure = 101325 Pa
  // T0 = 288.15 K (15°C)

  const float P0 = 101325.0f;  // Sea level pressure (Pa)
  const float EXPONENT = 1.0f / 5.255f;

  if (pressure_pa <= 0.0f) {
    return 0.0f;  // Safety check
  }

  float ratio = pressure_pa / P0;
  float altitude_m = 44330.0f * (1.0f - powf(ratio, EXPONENT));

  return altitude_m;
}
