#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include <cstdint>

// ============================================================================
// PWM Output Controller for ESCs
// ============================================================================
// Generates 50 Hz PWM on PA0-PA3 (Motors 1-4)
// PWM pulse width range: 1000-2000 μs (ESC standard)
// 50 Hz = 20 ms period
//
// STM32F4 Timer Configuration:
//   Timer 2 (TIM2) on PA0-PA3 (channels 1-4)
//   Frequency: 50 Hz (20 ms period)
//   Resolution: 1 μs per tick (requires 1 MHz timer clock)
//   PWM values: 1000-2000 (directly in microseconds)

class PWMController {
public:
  PWMController();

  // Initialize PWM output on PA0-PA3 (Motors 1-4)
  // Configures Timer 2 for 50 Hz PWM, 1000-2000 μs pulse width
  void begin();

  // Set pulse width for a specific motor (1-4)
  // pulse_width: 1000-2000 μs
  // Returns: true if motor index is valid, false otherwise
  bool setMotor(uint8_t motor_index, uint16_t pulse_width);

  // Set pulse width for all motors at once
  // Values: 1000-2000 μs each
  void setAllMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

  // Disarm all motors (set to 1000 μs - stopped state)
  // Safe to call at any time
  void disarmAll();

  // Get current pulse width for a motor
  uint16_t getMotor(uint8_t motor_index) const;

  // Check if PWM output is initialized
  bool isInitialized() const { return initialized; }

private:
  bool initialized;
  uint16_t current_pulse[4];  // Current pulse width for each motor (1-4)

  // Helper: Clamp pulse width to 1000-2000 μs
  uint16_t clamp_pulse(uint16_t pulse);

  // Helper: Configure STM32F4 Timer 2 for 50 Hz PWM
  void configure_timer2();
};

#endif // PWM_OUTPUT_H
