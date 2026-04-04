#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// PID Controller Gain Configuration for Bench Testing
// ============================================================================
// These gains are tuned for stable roll/pitch/yaw control on the bench
// without propellers. Adjust based on observed behavior during testing.

// Rate PID Controller - Inner loop (gyroscope feedback)
// Controls angular velocity in rad/s
#define RATE_PID_KP 0.015f     // Proportional gain (reduce if oscillating)
#define RATE_PID_KI 0.02f      // Integral gain (anti-windup)
#define RATE_PID_KD 0.0f       // Derivative gain (can cause noise)

// Attitude PID Controller - Outer loop (accelerometer feedback)
// Converts desired angles to desired rates for rate controller
#define ATT_PID_KP 4.5f        // Proportional gain (reduce if oscillating)
#define ATT_PID_KI 0.05f       // Integral gain (angle hold)
#define ATT_PID_KD 0.15f       // Derivative gain (damping)

// Altitude PID Controller
// Maintains altitude hold by adjusting throttle
#define ALT_PID_KP 0.3f        // Proportional gain
#define ALT_PID_KI 0.05f       // Integral gain
#define ALT_PID_KD 0.1f        // Derivative gain

// ============================================================================
// Madgwick Filter — Quaternion-Based Sensor Fusion (Phase 2.2)
// ============================================================================
//
// Uncomment to enable Madgwick filter for drift-free yaw using magnetometer
// When disabled: uses complementary filter (pure gyro yaw drifts)
// When enabled: magnetometer corrects yaw, no drift
//
// Tuning parameter (MADGWICK_BETA):
//   0.033 = very slow, smooth (heavy gyro reliance, slow mag correction)
//   0.1   = default, balanced (good for outdoor with stable mag field)
//   0.5   = fast, responsive (trusts accel/mag more, less gyro)
//
// #define USE_MADGWICK          // DISABLED for debugging — yaw tracking issue
#define MADGWICK_BETA        0.1f    // Gradient descent gain (0.033–0.5 typical)
#define MADGWICK_SAMPLE_RATE 100.0f  // Hz — must match main loop rate

// ============================================================================
// MAVLink Telemetry — QGroundControl integration
// ============================================================================
//
// Uncomment to enable MAVLink output over USB Serial.
// Requires: install "MAVLink" library from Arduino Library Manager.
//
// When ENABLED:  Serial outputs MAVLink binary → connect QGC to USB COM port
// When DISABLED: Serial outputs human-readable text dashboard (default)
//
// To enable: uncomment the line below, flash, open QGroundControl
// #define MAVLINK_ENABLED

// ============================================================================
// Debug Logging Configuration
// ============================================================================

// Enable debug output (print every N loop iterations)
// Only active when MAVLINK_ENABLED is NOT defined
#define DEBUG_LOG_INTERVAL 500  // Print every 500 loops (~5s at 100 Hz)

// Disable GPS debug output for clean serial output
#define DISABLE_GPS_DEBUG    // Comment out to see raw NMEA + GPS status lines

// Format string for debug output:
// "T:1234 | Att: R=0.5 P=1.2 Y=-0.3 | RC: Th=1500 | Motors: 1500 1500 1500 1500"

#endif // CONFIG_H
