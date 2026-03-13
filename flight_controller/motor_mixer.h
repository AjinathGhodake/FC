#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <Arduino.h>
#include <cstdint>
#include <algorithm>

// ============================================================================
// Motor Output Structure
// ============================================================================

struct MotorOutput {
  uint16_t m1;  // Front-Left motor PWM (1000-2000 μs)
  uint16_t m2;  // Front-Right motor PWM (1000-2000 μs)
  uint16_t m3;  // Rear-Right motor PWM (1000-2000 μs)
  uint16_t m4;  // Rear-Left motor PWM (1000-2000 μs)
};

// ============================================================================
// Motor Mixer Class
// ============================================================================

class MotorMixer {
public:
  MotorMixer();

  // Mix motor outputs for X-frame quadcopter
  // Input:
  //   throttle:     0-100% (0 = no thrust, 100 = max thrust)
  //   roll:         -100..+100 (rate control correction, -100 = roll left, +100 = roll right)
  //   pitch:        -100..+100 (rate control correction, -100 = pitch forward, +100 = pitch back)
  //   yaw:          -100..+100 (rate control correction, -100 = yaw left, +100 = yaw right)
  //   altitude_trim:-100..+100 (altitude controller trim adjustment)
  //
  // Output:
  //   MotorOutput struct with m1-m4 as 1000-2000 μs PWM values
  //
  // X-frame mixing matrix:
  //   M1 (FL) = throttle + pitch + roll + yaw
  //   M2 (FR) = throttle + pitch - roll - yaw
  //   M3 (RR) = throttle - pitch - roll + yaw
  //   M4 (RL) = throttle - pitch + roll - yaw
  MotorOutput mix(float throttle, float roll, float pitch, float yaw, float altitude_trim);

  // Set throttle limits (min and max PWM values)
  // Default: 1000 (min) to 2000 (max)
  void setThrottleLimits(uint16_t min_pwm, uint16_t max_pwm);

  // Get last computed motor output (for debugging)
  MotorOutput getLastOutput() const;

private:
  uint16_t pwm_min;      // Minimum PWM (idle throttle, default 1000)
  uint16_t pwm_max;      // Maximum PWM (max throttle, default 2000)
  MotorOutput last_output;  // Last computed output (for debugging)

  // Helper: Convert normalized motor value (-100..+100) to PWM (1000-2000 μs)
  uint16_t normalize_to_pwm(float normalized_value);
};

#endif // MOTOR_MIXER_H
