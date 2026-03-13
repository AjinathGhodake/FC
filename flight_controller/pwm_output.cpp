#include <Arduino.h>
#include <Servo.h>
#include "pwm_output.h"

// Use Servo library for ESC PWM control (50 Hz, 1000-2000 μs)
// Motor pins (avoids PA2/PA3 conflict with UART2/ELRS):
//   Motor 1 (FL): PA0
//   Motor 2 (FR): PA1
//   Motor 3 (RR): PB0
//   Motor 4 (RL): PB1

static Servo motor[4];
static const uint8_t motor_pins[4] = {PA0, PA1, PB0, PB1};

PWMController::PWMController() : initialized(false) {
  for (int i = 0; i < 4; i++) {
    current_pulse[i] = 1000;
  }
}

void PWMController::begin() {
  for (int i = 0; i < 4; i++) {
    motor[i].attach(motor_pins[i], 1000, 2000);  // min 1000μs, max 2000μs
    motor[i].writeMicroseconds(1000);             // motors off
  }
  initialized = true;
}

bool PWMController::setMotor(uint8_t motor_index, uint16_t pulse_width) {
  if (motor_index < 1 || motor_index > 4) return false;

  pulse_width = clamp_pulse(pulse_width);
  current_pulse[motor_index - 1] = pulse_width;
  motor[motor_index - 1].writeMicroseconds(pulse_width);
  return true;
}

void PWMController::setAllMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  setMotor(1, m1);
  setMotor(2, m2);
  setMotor(3, m3);
  setMotor(4, m4);
}

void PWMController::disarmAll() {
  setAllMotors(1000, 1000, 1000, 1000);
}

uint16_t PWMController::getMotor(uint8_t motor_index) const {
  if (motor_index < 1 || motor_index > 4) return 0;
  return current_pulse[motor_index - 1];
}

uint16_t PWMController::clamp_pulse(uint16_t pulse) {
  if (pulse < 1000) return 1000;
  if (pulse > 2000) return 2000;
  return pulse;
}
