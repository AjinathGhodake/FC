#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include <Arduino.h>

// ============================================================================
// PWM Output Controller for ESCs
// ============================================================================
// Motor pins (PA2/PA3 reserved for UART2/ELRS):
//   Motor 1 (FL): PA0 - TIM2_CH1
//   Motor 2 (FR): PA1 - TIM2_CH2
//   Motor 3 (RR): PB0 - TIM3_CH3
//   Motor 4 (RL): PB1 - TIM3_CH4
//
// PWM: 50 Hz (20 ms period), 1000-2000 us pulse width

class PWMController {
public:
  PWMController();

  // Initialize PWM output on PA0-PA3 (Motors 1-4)
  // Configures Timer 2 for 50 Hz PWM, 1000-2000 us pulse width
  void begin();

  // Set pulse width for a specific motor (1-4)
  // pulse_width: 1000-2000 us
  // Returns: true if motor index is valid, false otherwise
  bool setMotor(uint8_t motor_index, uint16_t pulse_width);

  // Set pulse width for all motors at once
  // Values: 1000-2000 us each
  void setAllMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

  // Disarm all motors (set to 1000 us - stopped state)
  // Safe to call at any time
  void disarmAll();

  // Get current pulse width for a motor
  uint16_t getMotor(uint8_t motor_index) const;

  // Check if PWM output is initialized
  bool isInitialized() const { return initialized; }

private:
  bool initialized;
  uint16_t current_pulse[4];  // Current pulse width for each motor (1-4)

  // Helper: Clamp pulse width to 1000-2000 us
  uint16_t clamp_pulse(uint16_t pulse);
};

#endif // PWM_OUTPUT_H
