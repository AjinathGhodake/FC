#include <Arduino.h>
#include "pwm_output.h"

// ============================================================================
// HardwareTimer-based PWM for TIM2 on PA0-PA3
// ============================================================================
// TIM2 channels:
//   PA0 = TIM2_CH1 (Motor 1)
//   PA1 = TIM2_CH2 (Motor 2)
//   PA2 = TIM2_CH3 (Motor 3)
//   PA3 = TIM2_CH4 (Motor 4)
//
// ESC standard: 50 Hz (20 ms period), 1000-2000 us pulse width

// Module-level pointer to the HardwareTimer instance
static HardwareTimer* pwmTimer = nullptr;

PWMController::PWMController() : initialized(false) {
  for (int i = 0; i < 4; i++) {
    current_pulse[i] = 1500;  // Default to neutral throttle
  }
}

// ============================================================================
// Public Methods
// ============================================================================

void PWMController::begin() {
  // Create HardwareTimer for TIM2
  TIM_TypeDef* instance = TIM2;
  pwmTimer = new HardwareTimer(instance);

  // Set overflow to 50 Hz (20 ms period) for ESC PWM
  pwmTimer->setOverflow(50, HERTZ_FORMAT);

  // Configure all 4 channels as PWM output
  // PA0 = Channel 1, PA1 = Channel 2, PA2 = Channel 3, PA3 = Channel 4
  pwmTimer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA0);
  pwmTimer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PA1);
  pwmTimer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PA2);
  pwmTimer->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PA3);

  // Set initial pulse to 1500 us (neutral throttle)
  for (int ch = 1; ch <= 4; ch++) {
    pwmTimer->setCaptureCompare(ch, 1500, MICROSEC_COMPARE_FORMAT);
  }

  // Start the timer
  pwmTimer->resume();

  initialized = true;
}

bool PWMController::setMotor(uint8_t motor_index, uint16_t pulse_width) {
  // Motor index: 1-4 (converted to 0-3 internally)
  if (motor_index < 1 || motor_index > 4) {
    return false;
  }

  pulse_width = clamp_pulse(pulse_width);
  current_pulse[motor_index - 1] = pulse_width;

  // Update the corresponding TIM2 channel via HardwareTimer API
  if (pwmTimer != nullptr) {
    pwmTimer->setCaptureCompare(motor_index, pulse_width, MICROSEC_COMPARE_FORMAT);
  }

  return true;
}

void PWMController::setAllMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  setMotor(1, m1);
  setMotor(2, m2);
  setMotor(3, m3);
  setMotor(4, m4);
}

void PWMController::disarmAll() {
  // Set all motors to 1000 us (stopped)
  setAllMotors(1000, 1000, 1000, 1000);
}

uint16_t PWMController::getMotor(uint8_t motor_index) const {
  if (motor_index < 1 || motor_index > 4) {
    return 0;
  }
  return current_pulse[motor_index - 1];
}

// ============================================================================
// Private Methods
// ============================================================================

uint16_t PWMController::clamp_pulse(uint16_t pulse) {
  if (pulse < 1000) return 1000;
  if (pulse > 2000) return 2000;
  return pulse;
}
