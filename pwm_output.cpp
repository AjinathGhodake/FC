#include "pwm_output.h"

// STM32F4 HAL macros (if not using full HAL library)
// These are direct register accesses for Timer 2 on PA0-PA3

// ============================================================================
// Timer 2 Configuration for STM32F4
// ============================================================================
// TIM2 on APB1 (84 MHz clock on STM32F401/F411)
// We want: 50 Hz (20 ms period)
// Resolution: 1 μs per tick
//
// Setup:
//   APB1 clock = 84 MHz
//   Prescaler (PSC) = 84 - 1 (divides 84 MHz to 1 MHz, giving 1 μs per tick)
//   Auto-reload (ARR) = 20000 - 1 (20000 * 1 μs = 20 ms period)
//   PWM values: 1000-2000 (directly in microseconds)

PWMController::PWMController() : initialized(false) {
  for (int i = 0; i < 4; i++) {
    current_pulse[i] = 1500;  // Default to neutral throttle
  }
}

// ============================================================================
// Public Methods
// ============================================================================

void PWMController::begin() {
  // Configure GPIO pins PA0-PA3 as alternate function (Timer 2 output)
  // PA0 = TIM2_CH1 (Motor 1)
  // PA1 = TIM2_CH2 (Motor 2)
  // PA2 = TIM2_CH3 (Motor 3)
  // PA3 = TIM2_CH4 (Motor 4)

  // Enable GPIOA and TIM2 clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // Enable TIM2 clock

  // Configure PA0-PA3 as alternate function for TIM2
  // MODER: 10 = alternate function
  GPIOA->MODER &= ~(0xFFU);                              // Clear PA0-PA3 MODER bits
  GPIOA->MODER |=  (0xAAU);                              // Set to alternate function (1010 pattern)

  // AFRL: Set alternate function to AF1 (TIM2)
  GPIOA->AFR[0] &= ~(0xFFFFU);                           // Clear PA0-PA3 alternate function
  GPIOA->AFR[0] |=  (0x1111U);                           // Set to AF1 for TIM2

  // Configure output type and speed
  GPIOA->OTYPER &= ~(0x0FU);                             // Push-pull output
  GPIOA->OSPEEDR |= (0xFFU);                             // High speed

  // Configure Timer 2
  configure_timer2();

  initialized = true;
}

bool PWMController::setMotor(uint8_t motor_index, uint16_t pulse_width) {
  // Motor index: 1-4 (converted to 0-3 internally)
  if (motor_index < 1 || motor_index > 4) {
    return false;
  }

  pulse_width = clamp_pulse(pulse_width);
  current_pulse[motor_index - 1] = pulse_width;

  // Update the corresponding TIM2 channel CCR register
  // TIM2->CCR1 = Motor 1 (PA0)
  // TIM2->CCR2 = Motor 2 (PA1)
  // TIM2->CCR3 = Motor 3 (PA2)
  // TIM2->CCR4 = Motor 4 (PA3)

  switch (motor_index) {
    case 1:
      TIM2->CCR1 = pulse_width;
      break;
    case 2:
      TIM2->CCR2 = pulse_width;
      break;
    case 3:
      TIM2->CCR3 = pulse_width;
      break;
    case 4:
      TIM2->CCR4 = pulse_width;
      break;
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
  // Set all motors to 1000 μs (stopped)
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

void PWMController::configure_timer2() {
  // Disable timer during configuration
  TIM2->CR1 &= ~TIM_CR1_CEN;

  // Set prescaler to get 1 MHz clock (84 MHz / 84 = 1 MHz)
  // PSC is 16-bit, value loaded is PSC + 1
  // We want PSC = 83 (so PSC + 1 = 84)
  TIM2->PSC = 83;

  // Set auto-reload register for 50 Hz (20 ms period)
  // ARR = 20000 - 1 = 19999 (20000 counts at 1 MHz = 20 ms)
  TIM2->ARR = 19999;

  // Configure all 4 channels in PWM mode 1 (output high when counter < CCR)
  // CCMR1: Channels 1 and 2
  // CCMR2: Channels 3 and 4

  // Channel 1 (PA0): PWM mode 1, preload enabled
  TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE);
  TIM2->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

  // Channel 2 (PA1): PWM mode 1, preload enabled
  TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC2PE);
  TIM2->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

  // Channel 3 (PA2): PWM mode 1, preload enabled
  TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC3PE);
  TIM2->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

  // Channel 4 (PA3): PWM mode 1, preload enabled
  TIM2->CCMR2 &= ~(TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE);
  TIM2->CCMR2 |= (0x6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

  // Enable output on all channels (CCER register)
  TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

  // Enable auto-reload preload and enable timer
  TIM2->CR1 |= (TIM_CR1_ARPE | TIM_CR1_CEN);

  // Generate update event to load PSC and ARR
  TIM2->EGR |= TIM_EGR_UG;

  // Set initial compare values to neutral throttle (1500 μs)
  TIM2->CCR1 = 1500;
  TIM2->CCR2 = 1500;
  TIM2->CCR3 = 1500;
  TIM2->CCR4 = 1500;
}
