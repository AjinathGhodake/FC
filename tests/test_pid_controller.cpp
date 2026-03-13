#include "../pid_controller.h"
#include <cstdio>
#include <cassert>
#include <cmath>

// Tolerance for floating point comparisons
const float TOLERANCE = 0.1f;

bool almostEqual(float a, float b, float tol = TOLERANCE) {
  return std::abs(a - b) < tol;
}

// Test 1: Proportional response to step input
// When desired > actual, output should increase proportionally
void test_proportional_response() {
  printf("Test 1: Proportional response to step input...\n");

  PIDController pid(1.0f, 0.0f, 0.0f);  // Kp=1.0, Ki=0, Kd=0 (pure P)
  pid.setOutputLimits(-100.0f, 100.0f);

  // Initial state: desired = 10.0, actual = 0.0
  // With Kp=1.0, output should be approximately 10.0
  float output = pid.update(10.0f, 0.0f, 0.01f);
  printf("  Desired=10.0, Actual=0.0, Output=%.2f\n", output);
  assert(almostEqual(output, 10.0f, 0.5f));

  // After correction is applied (simulated), actual converges toward desired
  // Next step: desired = 10.0, actual = 5.0 (halfway there)
  output = pid.update(10.0f, 5.0f, 0.01f);
  printf("  Desired=10.0, Actual=5.0, Output=%.2f\n", output);
  assert(almostEqual(output, 5.0f, 0.5f));

  // Finally: desired = 10.0, actual = 10.0 (error = 0)
  output = pid.update(10.0f, 10.0f, 0.01f);
  printf("  Desired=10.0, Actual=10.0, Output=%.2f\n", output);
  assert(almostEqual(output, 0.0f, 0.5f));

  printf("  ✓ PASSED\n\n");
}

// Test 2: Integral response over time
// Integral should accumulate and help reduce steady-state error
void test_integral_response() {
  printf("Test 2: Integral response over time...\n");

  PIDController pid(0.0f, 1.0f, 0.0f);  // Kp=0, Ki=1.0, Kd=0 (pure I)
  pid.setOutputLimits(-100.0f, 100.0f);

  // Simulate persistent error of 5.0 rad/s for several time steps
  float output = 0.0f;
  for (int i = 0; i < 5; i++) {
    output = pid.update(5.0f, 0.0f, 0.01f);
    printf("  Step %d: Output=%.4f, Integral=%.4f\n", i, output, pid.getIntegral());
  }

  // After 5 steps of 0.01s with error=5.0, integral should grow
  // integral = 5.0 * 0.01 * 5 = 0.25, output = Ki * integral = 0.25
  printf("  Final integral value: %.4f\n", pid.getIntegral());
  assert(almostEqual(pid.getIntegral(), 0.25f, 0.01f));
  assert(almostEqual(output, 0.25f, 0.01f));

  // Test anti-windup: continuous error should clamp at output limits
  pid.reset();
  for (int i = 0; i < 200; i++) {
    output = pid.update(100.0f, 0.0f, 0.01f);  // Large error
  }
  printf("  After saturation, Output=%.2f (should be at limit 100.0)\n", output);
  assert(almostEqual(output, 100.0f, 0.5f));

  printf("  ✓ PASSED\n\n");
}

// Test 3: RatePIDController with all three axes
void test_rate_pid_controller() {
  printf("Test 3: RatePIDController (all axes)...\n");

  RatePIDController rate_pid;

  // Scenario: Pilot commands roll rate of 5.0 rad/s, but actual is 0.0
  // PID should output a correction
  RatePIDController::RateCorrection correction = rate_pid.update(
      5.0f, 0.0f,    // desired_roll, actual_roll
      0.0f, 0.0f,    // desired_pitch, actual_pitch
      0.0f, 0.0f,    // desired_yaw, actual_yaw
      0.01f           // dt
  );

  printf("  Roll desired=5.0, actual=0.0:\n");
  printf("    Roll correction:  %.2f\n", correction.roll);
  printf("    Pitch correction: %.2f\n", correction.pitch);
  printf("    Yaw correction:   %.2f\n", correction.yaw);

  // Roll should have a significant correction (error is 5.0)
  assert(correction.roll > 0.0f);
  assert(correction.pitch < 0.1f);  // No pitch error
  assert(correction.yaw < 0.1f);    // No yaw error

  printf("  ✓ PASSED\n\n");
}

// Test 4: Anti-windup prevents unbounded integral growth
void test_anti_windup() {
  printf("Test 4: Anti-windup clamps integral accumulation...\n");

  PIDController pid(0.1f, 1.0f, 0.0f);  // Kp=0.1, Ki=1.0
  pid.setOutputLimits(-100.0f, 100.0f);

  // Simulate persistent error that would normally accumulate unboundedly
  float output = 0.0f;
  for (int i = 0; i < 500; i++) {
    output = pid.update(100.0f, 0.0f, 0.01f);  // Constant error of 100.0
  }

  printf("  After 500 steps with Ki=1.0, error=100:\n");
  printf("    Output:   %.2f\n", output);
  printf("    Integral: %.2f\n", pid.getIntegral());

  // Without anti-windup, integral would be huge; with it, clamped
  // Output should be at limit ±100.0
  assert(almostEqual(output, 100.0f, 0.5f));
  assert(pid.getIntegral() < 150.0f);  // Should be bounded

  printf("  ✓ PASSED\n\n");
}

int main() {
  printf("=== PID Controller Tests ===\n\n");

  test_proportional_response();
  test_integral_response();
  test_rate_pid_controller();
  test_anti_windup();

  printf("=== All tests passed! ===\n");
  return 0;
}
