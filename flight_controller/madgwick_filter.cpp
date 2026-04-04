#include "madgwick_filter.h"

MadgwickFilter::MadgwickFilter(float beta, float sample_rate)
  : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), beta(beta) {
  inv_sample_freq = 1.0f / sample_rate;
}

void MadgwickFilter::update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float mx, float my, float mz) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float b_x, b_z;

  // Normalize accelerometer (magnitude should be ~9.81 m/s²)
  recipNorm = invSqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // Normalize magnetometer
  recipNorm = invSqrt(mx * mx + my * my + mz * mz);
  mx *= recipNorm;
  my *= recipNorm;
  mz *= recipNorm;

  // Auxiliary variables to avoid repeated arithmetic
  // Reference frame magnetic field components
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
  hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
  b_x = sqrtf(hx * hx + hy * hy);
  b_z = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

  // Estimated direction of gravity and magnetic field
  s0 = -q2 * (2.0f * q1q3 - 2.0f * q0q2 - ax) + q1 * (2.0f * q0q3 + 2.0f * q1q2 - ay) - q0 * (ay);
  s0 += -2.0f * b_z * q2 * (2.0f * b_x * (0.5f - q2q2 - q3q3) + 2.0f * b_z * (q1q3 - q0q2) - mx)
        + 2.0f * b_x * q3 * (2.0f * b_x * (q1q2 - q0q3) + 2.0f * b_z * (q0q1 + q2q3) - my)
        + 2.0f * b_x * q2 * (2.0f * b_x * (0.5f - q1q1 - q3q3) + 2.0f * b_z * (q2q3 - q0q1) - mz);

  s1 = q3 * (2.0f * q1q3 - 2.0f * q0q2 - ax) + q0 * (2.0f * q0q3 + 2.0f * q1q2 - ay) - q2 * (ay);
  s1 += -2.0f * b_z * q1 * (2.0f * b_x * (0.5f - q2q2 - q3q3) + 2.0f * b_z * (q1q3 - q0q2) - mx)
        - 2.0f * b_x * q2 * (2.0f * b_x * (q1q2 - q0q3) + 2.0f * b_z * (q0q1 + q2q3) - my)
        + 2.0f * b_x * q3 * (2.0f * b_x * (0.5f - q1q1 - q3q3) + 2.0f * b_z * (q2q3 - q0q1) - mz);

  s2 = q0 * (2.0f * q1q3 - 2.0f * q0q2 - ax) + q3 * (2.0f * q0q3 + 2.0f * q1q2 - ay) - q1 * (az);
  s2 += 2.0f * b_z * q3 * (2.0f * b_x * (0.5f - q2q2 - q3q3) + 2.0f * b_z * (q1q3 - q0q2) - mx)
       + 2.0f * b_x * q1 * (2.0f * b_x * (q1q2 - q0q3) + 2.0f * b_z * (q0q1 + q2q3) - my)
       - 2.0f * b_x * q0 * (2.0f * b_x * (0.5f - q1q1 - q3q3) + 2.0f * b_z * (q2q3 - q0q1) - mz);

  s3 = q1 * (2.0f * q0q3 + 2.0f * q1q2 - ay) - q0 * (2.0f * q1q3 - 2.0f * q0q2 - ax) - q3 * (az);
  s3 += -2.0f * b_z * q0 * (2.0f * b_x * (0.5f - q2q2 - q3q3) + 2.0f * b_z * (q1q3 - q0q2) - mx)
       + 2.0f * b_x * q1 * (2.0f * b_x * (q1q2 - q0q3) + 2.0f * b_z * (q0q1 + q2q3) - my)
       + 2.0f * b_x * q2 * (2.0f * b_x * (0.5f - q1q1 - q3q3) + 2.0f * b_z * (q2q3 - q0q1) - mz);

  // Normalize step magnitude
  recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  s0 *= recipNorm;
  s1 *= recipNorm;
  s2 *= recipNorm;
  s3 *= recipNorm;

  // Compute quaternion derivative
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

  // Integrate to yield quaternion
  float dt = inv_sample_freq;
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalize quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

float MadgwickFilter::getRoll() const {
  return atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
}

float MadgwickFilter::getPitch() const {
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  sinp = (sinp > 1.0f) ? 1.0f : (sinp < -1.0f) ? -1.0f : sinp;
  return asinf(sinp);
}

float MadgwickFilter::getYaw() const {
  return atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
}

void MadgwickFilter::reset() {
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
}

void MadgwickFilter::getQuaternion(float& q0_out, float& q1_out, float& q2_out, float& q3_out) const {
  q0_out = q0;
  q1_out = q1;
  q2_out = q2;
  q3_out = q3;
}

// Fast inverse square root using Newton-Raphson iteration
float MadgwickFilter::invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - halfx * y * y);  // 1st iteration
  y = y * (1.5f - halfx * y * y);  // 2nd iteration (for better precision)
  return y;
}
