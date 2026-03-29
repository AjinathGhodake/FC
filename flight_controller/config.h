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
// Debug Logging Configuration
// ============================================================================

// Enable debug output (print every N loop iterations)
#define DEBUG_LOG_INTERVAL 300  // Print every 300 loops (~3s at 100 Hz)

// Format string for debug output:
// "T:1234 | Att: R=0.5 P=1.2 Y=-0.3 | RC: Th=1500 | Motors: 1500 1500 1500 1500"

#endif // CONFIG_H
