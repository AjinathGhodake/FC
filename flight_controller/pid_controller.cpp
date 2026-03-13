#include <Arduino.h>
#include "pid_controller.h"
#include <cmath>
#include <algorithm>

// ============================================================================
// PIDController Implementation
// ============================================================================

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), integral(0.0f), prev_error(0.0f),
      output_min(DEFAULT_MIN), output_max(DEFAULT_MAX) {}

float PIDController::update(float desired, float actual, float dt) {
  if (dt <= 0.0f) {
    return 0.0f;  // Invalid time delta
  }

  // Calculate error
  float error = desired - actual;

  // Proportional term
  float p_term = kp * error;

  // Integral term with anti-windup (clamp integral before accumulation)
  integral += error * dt;
  // Clamp integral to prevent windup
  integral = std::clamp(integral, output_min / ki, output_max / ki);
  float i_term = ki * integral;

  // Derivative term
  float d_term = kd * (error - prev_error) / dt;
  prev_error = error;

  // Sum all terms
  float output = p_term + i_term + d_term;

  // Clamp output to limits
  output = std::clamp(output, output_min, output_max);

  return output;
}

void PIDController::reset() {
  integral = 0.0f;
  prev_error = 0.0f;
}

void PIDController::setOutputLimits(float min_out, float max_out) {
  output_min = min_out;
  output_max = max_out;
}

float PIDController::getIntegral() const {
  return integral;
}

float PIDController::getLastError() const {
  return prev_error;
}

// ============================================================================
// RatePIDController Implementation
// ============================================================================

// Default gains for rate control (tuned for typical quadcopter)
// These are starting points; actual tuning depends on hardware
static constexpr float DEFAULT_KP_RP = 0.15f;   // Roll/pitch proportional
static constexpr float DEFAULT_KI_RP = 0.08f;   // Roll/pitch integral
static constexpr float DEFAULT_KD_RP = 0.04f;   // Roll/pitch derivative

static constexpr float DEFAULT_KP_YAW = 0.30f;  // Yaw proportional (more responsive)
static constexpr float DEFAULT_KI_YAW = 0.10f;  // Yaw integral
static constexpr float DEFAULT_KD_YAW = 0.02f;  // Yaw derivative

RatePIDController::RatePIDController()
    : roll_pid(DEFAULT_KP_RP, DEFAULT_KI_RP, DEFAULT_KD_RP),
      pitch_pid(DEFAULT_KP_RP, DEFAULT_KI_RP, DEFAULT_KD_RP),
      yaw_pid(DEFAULT_KP_YAW, DEFAULT_KI_YAW, DEFAULT_KD_YAW) {
  // Set output limits for motor control normalization
  roll_pid.setOutputLimits(-100.0f, 100.0f);
  pitch_pid.setOutputLimits(-100.0f, 100.0f);
  yaw_pid.setOutputLimits(-100.0f, 100.0f);
}

RatePIDController::RateCorrection RatePIDController::update(
    float desired_roll_rate, float actual_roll_rate,
    float desired_pitch_rate, float actual_pitch_rate,
    float desired_yaw_rate, float actual_yaw_rate,
    float dt) {

  RateCorrection correction;
  correction.roll = roll_pid.update(desired_roll_rate, actual_roll_rate, dt);
  correction.pitch = pitch_pid.update(desired_pitch_rate, actual_pitch_rate, dt);
  correction.yaw = yaw_pid.update(desired_yaw_rate, actual_yaw_rate, dt);

  return correction;
}

void RatePIDController::reset() {
  roll_pid.reset();
  pitch_pid.reset();
  yaw_pid.reset();
}

void RatePIDController::setGains(float kp_rp, float ki_rp, float kd_rp,
                                   float kp_yaw, float ki_yaw, float kd_yaw) {
  // Create new PID controllers with the specified gains
  roll_pid = PIDController(kp_rp, ki_rp, kd_rp);
  pitch_pid = PIDController(kp_rp, ki_rp, kd_rp);
  yaw_pid = PIDController(kp_yaw, ki_yaw, kd_yaw);

  // Restore output limits
  roll_pid.setOutputLimits(-100.0f, 100.0f);
  pitch_pid.setOutputLimits(-100.0f, 100.0f);
  yaw_pid.setOutputLimits(-100.0f, 100.0f);
}

// ============================================================================
// AttitudePIDController Implementation
// ============================================================================

// Default gains for attitude control (convert angle error to rate setpoint)
// Higher gains make the drone respond faster to attitude commands
static constexpr float DEFAULT_KP_ATTITUDE = 4.5f;    // Proportional
static constexpr float DEFAULT_KI_ATTITUDE = 0.05f;   // Integral (small to avoid windup)
static constexpr float DEFAULT_KD_ATTITUDE = 0.15f;   // Derivative (smooth response)

AttitudePIDController::AttitudePIDController()
    : roll_pid(DEFAULT_KP_ATTITUDE, DEFAULT_KI_ATTITUDE, DEFAULT_KD_ATTITUDE),
      pitch_pid(DEFAULT_KP_ATTITUDE, DEFAULT_KI_ATTITUDE, DEFAULT_KD_ATTITUDE),
      yaw_pid(DEFAULT_KP_ATTITUDE, DEFAULT_KI_ATTITUDE, DEFAULT_KD_ATTITUDE) {
  // Set output limits for angular rate (rad/s)
  // Typical max rate for a drone: ~3 rad/s (170 deg/s)
  float max_rate = 3.0f;  // rad/s
  roll_pid.setOutputLimits(-max_rate, max_rate);
  pitch_pid.setOutputLimits(-max_rate, max_rate);
  yaw_pid.setOutputLimits(-max_rate, max_rate);
}

AttitudePIDController::Output AttitudePIDController::update(
    float roll_des, float pitch_des, float yaw_des,
    float roll_act, float pitch_act, float yaw_act, float dt) {

  Output output;
  output.roll_rate = roll_pid.update(roll_des, roll_act, dt);
  output.pitch_rate = pitch_pid.update(pitch_des, pitch_act, dt);
  output.yaw_rate = yaw_pid.update(yaw_des, yaw_act, dt);

  return output;
}

void AttitudePIDController::reset() {
  roll_pid.reset();
  pitch_pid.reset();
  yaw_pid.reset();
}

void AttitudePIDController::setGains(float kp, float ki, float kd) {
  // Create new PID controllers with the specified gains
  roll_pid = PIDController(kp, ki, kd);
  pitch_pid = PIDController(kp, ki, kd);
  yaw_pid = PIDController(kp, ki, kd);

  // Restore output limits
  float max_rate = 3.0f;  // rad/s
  roll_pid.setOutputLimits(-max_rate, max_rate);
  pitch_pid.setOutputLimits(-max_rate, max_rate);
  yaw_pid.setOutputLimits(-max_rate, max_rate);
}

// ============================================================================
// AltitudeController Implementation
// ============================================================================

// Default gains for altitude control (barometer-based hover)
// These are initial tuning values; fine-tune based on flight testing
static constexpr float DEFAULT_KP_ALT = 0.3f;   // Proportional
static constexpr float DEFAULT_KI_ALT = 0.05f;  // Integral
static constexpr float DEFAULT_KD_ALT = 0.1f;   // Derivative

AltitudeController::AltitudeController()
    : pid(DEFAULT_KP_ALT, DEFAULT_KI_ALT, DEFAULT_KD_ALT) {
  // Set output limits: throttle adjustment from -100 to +100
  pid.setOutputLimits(-100.0f, 100.0f);
}

float AltitudeController::update(float desired_altitude, float actual_altitude, float dt) {
  // The PID controller will output a throttle adjustment
  // Positive adjustment = increase throttle (go higher)
  // Negative adjustment = decrease throttle (go lower)
  return pid.update(desired_altitude, actual_altitude, dt);
}

void AltitudeController::reset() {
  pid.reset();
}

void AltitudeController::setGains(float kp, float ki, float kd) {
  pid = PIDController(kp, ki, kd);
  // Restore output limits
  pid.setOutputLimits(-100.0f, 100.0f);
}

float AltitudeController::getIntegral() const {
  return pid.getIntegral();
}
