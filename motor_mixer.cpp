#include "motor_mixer.h"

// ============================================================================
// MotorMixer Implementation
// ============================================================================

MotorMixer::MotorMixer()
    : pwm_min(1000), pwm_max(2000) {
  // Initialize last output to neutral
  last_output.m1 = 1500;
  last_output.m2 = 1500;
  last_output.m3 = 1500;
  last_output.m4 = 1500;
}

void MotorMixer::setThrottleLimits(uint16_t min_pwm, uint16_t max_pwm) {
  pwm_min = min_pwm;
  pwm_max = max_pwm;
}

MotorOutput MotorMixer::getLastOutput() const {
  return last_output;
}

uint16_t MotorMixer::normalize_to_pwm(float normalized_value) {
  // Clamp input to ±100
  normalized_value = std::clamp(normalized_value, -100.0f, 100.0f);

  // Map -100..+100 to pwm_min..pwm_max
  // 0 -> center (1500 μs)
  // +100 -> pwm_max (2000 μs)
  // -100 -> pwm_min (1000 μs)
  float pwm_range = pwm_max - pwm_min;
  float center_pwm = pwm_min + pwm_range / 2.0f;

  float pwm_value = center_pwm + (normalized_value / 100.0f) * (pwm_range / 2.0f);

  // Clamp to valid PWM range
  pwm_value = std::clamp(pwm_value, (float)pwm_min, (float)pwm_max);

  return (uint16_t)pwm_value;
}

MotorOutput MotorMixer::mix(float throttle, float roll, float pitch, float yaw, float altitude_trim) {
  // Clamp all inputs to valid ranges
  throttle = std::clamp(throttle, 0.0f, 100.0f);          // 0-100%
  roll = std::clamp(roll, -100.0f, 100.0f);              // ±100
  pitch = std::clamp(pitch, -100.0f, 100.0f);            // ±100
  yaw = std::clamp(yaw, -100.0f, 100.0f);                // ±100
  altitude_trim = std::clamp(altitude_trim, -100.0f, 100.0f);  // ±100

  // Scale altitude_trim into throttle adjustment
  // altitude_trim ranges from -100 to +100
  // We'll scale it to ±20% of throttle range to avoid over-correction
  float throttle_adjustment = altitude_trim * 0.2f;
  float adjusted_throttle = throttle + throttle_adjustment;
  adjusted_throttle = std::clamp(adjusted_throttle, 0.0f, 100.0f);

  // Convert throttle (0-100%) to a base PWM offset from center
  // 0% throttle = 1000 μs (motor off)
  // 50% throttle = 1500 μs (hover)
  // 100% throttle = 2000 μs (max thrust)
  float throttle_pwm_offset = adjusted_throttle;  // Maps 0-100 to 1000-2000

  // Scale control corrections from -100..+100 to -50..+50 range
  // This prevents motor saturation and allows all motors to contribute
  float roll_scaled = roll * 0.5f;       // -50..+50
  float pitch_scaled = pitch * 0.5f;     // -50..+50
  float yaw_scaled = yaw * 0.5f;         // -50..+50

  // X-frame mixing matrix
  // M1 (FL) = throttle + pitch + roll + yaw
  // M2 (FR) = throttle + pitch - roll - yaw
  // M3 (RR) = throttle - pitch - roll + yaw
  // M4 (RL) = throttle - pitch + roll - yaw

  float m1_normalized = throttle_pwm_offset + pitch_scaled + roll_scaled + yaw_scaled;
  float m2_normalized = throttle_pwm_offset + pitch_scaled - roll_scaled - yaw_scaled;
  float m3_normalized = throttle_pwm_offset - pitch_scaled - roll_scaled + yaw_scaled;
  float m4_normalized = throttle_pwm_offset - pitch_scaled + roll_scaled - yaw_scaled;

  // Convert normalized values to PWM
  last_output.m1 = normalize_to_pwm(m1_normalized);
  last_output.m2 = normalize_to_pwm(m2_normalized);
  last_output.m3 = normalize_to_pwm(m3_normalized);
  last_output.m4 = normalize_to_pwm(m4_normalized);

  return last_output;
}
