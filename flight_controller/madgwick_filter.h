#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <cmath>

// Madgwick AHRS (Attitude and Heading Reference System) Filter
// Author: Sebastian Madgwick, 2010
// Reference: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Fuses gyroscope, accelerometer, and magnetometer data using a quaternion
// to produce drift-free roll, pitch, and yaw angles.
//
// Key features:
// - Quaternion representation (q0, q1, q2, q3)
// - Gyro integration for fast response
// - Accel/mag gradient descent for drift correction
// - No matrix multiplications — computationally efficient

class MadgwickFilter {
public:
  // Constructor
  // beta: algorithm gain for gradient descent (0.033 = slow/smooth, 0.1 = default, 0.5 = fast/noisy)
  // sample_rate: loop frequency in Hz (must match control loop rate)
  MadgwickFilter(float beta = 0.1f, float sample_rate = 100.0f);

  // Update filter with new sensor data
  // gx, gy, gz: gyroscope angular rates in rad/s
  // ax, ay, az: accelerometer readings in m/s² (raw, not normalized)
  // mx, my, mz: magnetometer readings in Gauss
  void update(float gx, float gy, float gz,
              float ax, float ay, float az,
              float mx, float my, float mz);

  // Get estimated Euler angles
  float getRoll() const;   // radians
  float getPitch() const;  // radians
  float getYaw() const;    // radians

  // Reset filter to initial state (quaternion = [1, 0, 0, 0])
  void reset();

  // Access quaternion components (for advanced use)
  void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;

private:
  // Quaternion state
  float q0, q1, q2, q3;

  // Filter parameters
  float beta;              // Algorithm gain
  float inv_sample_freq;   // 1 / sample_rate (for dt calculation)

  // Helper functions
  float invSqrt(float x);  // Fast inverse square root
  void computeGradient(float ax, float ay, float az,
                       float mx, float my, float mz,
                       float& gx, float& gy, float& gz);
};

#endif // MADGWICK_FILTER_H
