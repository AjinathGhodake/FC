#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// Single-axis PID controller with anti-windup
class PIDController {
public:
  PIDController(float kp, float ki, float kd);

  // Update PID with desired vs actual value and time delta
  // Returns the correction output
  float update(float desired, float actual, float dt);

  // Reset integral and previous error
  void reset();

  // Set output limits (e.g., ±100 for normalized control)
  void setOutputLimits(float min_out, float max_out);

  // Get current integral state (for debugging)
  float getIntegral() const;

  // Get last error (for debugging)
  float getLastError() const;

private:
  float kp;               // Proportional gain
  float ki;               // Integral gain
  float kd;               // Derivative gain
  float integral;         // Accumulated integral error
  float prev_error;       // Previous error (for derivative)
  float output_min;       // Minimum output limit
  float output_max;       // Maximum output limit

  static constexpr float DEFAULT_MIN = -100.0f;
  static constexpr float DEFAULT_MAX = 100.0f;
};

// Rate PID Controller for gyro-based stabilization
// Manages three separate PID controllers: one for each axis (roll, pitch, yaw)
class RatePIDController {
public:
  RatePIDController();

  // Update all three axes with desired and actual rates (rad/s)
  // Returns correction values for each axis
  struct RateCorrection {
    float roll;   // rad/s correction
    float pitch;  // rad/s correction
    float yaw;    // rad/s correction
  };

  RateCorrection update(float desired_roll_rate, float actual_roll_rate,
                        float desired_pitch_rate, float actual_pitch_rate,
                        float desired_yaw_rate, float actual_yaw_rate,
                        float dt);

  // Reset all controllers
  void reset();

  // Set gains for all axes (roll, pitch share gains; yaw separate)
  void setGains(float kp_rp, float ki_rp, float kd_rp,
                float kp_yaw, float ki_yaw, float kd_yaw);

private:
  PIDController roll_pid;
  PIDController pitch_pid;
  PIDController yaw_pid;
};

#endif // PID_CONTROLLER_H
