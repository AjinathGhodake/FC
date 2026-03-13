/*
 * Flight Controller Sketch for STM32F411 (Black Pill)
 *
 * Integrates:
 * - MPU-6050 IMU (accelerometer + gyroscope)
 * - BMP280 Barometer
 * - HMC5883L Magnetometer
 * - GPS NEO-6M
 * - CRSF RC Receiver (ELRS on UART2)
 *
 * Uses complementary filter for sensor fusion (accel + gyro -> roll/pitch/yaw)
 */

#include <Wire.h>
#include "sensors.h"
#include "../rc_input.h"
#include "../pid_controller.h"

// ============================================================================
// Configuration
// ============================================================================

#define LOOP_RATE_HZ 100      // Main loop frequency
#define LOOP_TIME_MS (1000.0f / LOOP_RATE_HZ)  // ~10 ms per loop
#define LOOP_TIME_S  (LOOP_TIME_MS / 1000.0f)  // ~0.01 s per loop

// LED pin
#define LED_PIN PC13

// BMP280 I2C address (SDO connected to GND = 0x76)
#define BMP280_ADDRESS 0x76

// ============================================================================
// Global Variables
// ============================================================================

ComplementaryFilter filter(0.98f);  // 98% gyro, 2% accel
IMUData imu_data;
Angles current_angles;

CRSFReceiver rc_receiver;  // CRSF RC receiver
RCChannels rc_channels;    // Current RC channel values

AttitudePIDController attitude_pid;  // Cascaded attitude controller
RatePIDController rate_pid;          // Inner rate controller
AltitudeController altitude_ctrl;    // Altitude hold controller

unsigned long last_loop_time = 0;
float loop_dt = LOOP_TIME_S;

// Altitude hold state
static float hover_altitude = 0.0f;  // Locked altitude when in alt-hold mode (meters)

// BMP280 calibration data (read from device)
// These are Bosch compensation coefficients for temperature and pressure
struct {
  uint16_t T1;
  int16_t T2, T3;
  uint16_t P1;
  int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calib;

// ============================================================================
// Function Prototypes
// ============================================================================

void setup_imu();
void setup_led();
void setup_rc_receiver();
void read_mpu6050();
void read_bmp280();
void update_filter();
void update_rc_receiver();
void update_attitude_controller();
void update_altitude_controller();
void print_debug_info();
void print_rc_info();

// ============================================================================
// Arduino Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial to stabilize

  Serial.println("Flight Controller Starting...");

  setup_led();
  setup_imu();
  setup_rc_receiver();

  Serial.println("Initialization complete.");
  Serial.println("Complementary Filter: 98% gyro + 2% accel");
  Serial.println("RC Receiver (CRSF/ELRS) initialized on UART2");

  last_loop_time = millis();
}

// ============================================================================
// Arduino Main Loop
// ============================================================================

void loop() {
  // Wait for next loop iteration to maintain 100 Hz rate
  unsigned long current_time = millis();
  unsigned long elapsed = current_time - last_loop_time;

  if (elapsed < LOOP_TIME_MS) {
    delay(LOOP_TIME_MS - elapsed);
  }

  last_loop_time = millis();
  loop_dt = (millis() - current_time) / 1000.0f;  // Actual dt

  // Read sensor data
  read_mpu6050();
  read_bmp280();

  // Update RC receiver
  update_rc_receiver();

  // Update complementary filter
  update_filter();

  // Get current angles (debugging)
  current_angles = filter.getAngles();

  // Get RC channels
  rc_channels = rc_receiver.getChannels();

  // Update cascaded attitude and rate controllers
  update_attitude_controller();

  // Update altitude hold controller
  update_altitude_controller();

  // Print debug info every 10 loops (1 second at 100 Hz)
  static int loop_counter = 0;
  if (++loop_counter >= 10) {
    print_debug_info();
    print_rc_info();
    loop_counter = 0;
  }

  // Toggle LED to indicate loop is running
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// ============================================================================
// Initialization Functions
// ============================================================================

void setup_led() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // OFF (active LOW)
}

void setup_imu() {
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz I2C clock

  // Enable internal pull-ups on I2C pins (PB6, PB7)
  // STM32F401/F411 I2C requires pull-ups for reliable operation
  GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));  // Clear PB6/PB7
  GPIOB->PUPDR |=  ((1 << 12) | (1 << 14));  // Set pull-up (1 = pull-up)

  delay(100);

  // Initialize MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up the MPU6050
  Wire.endTransmission();

  delay(100);

  // Set gyroscope range to ±250 deg/s (register 0x1B)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set accelerometer range to ±2g (register 0x1C)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);

  // Initialize BMP280 barometer
  // Read calibration data from registers 0x88-0xA1
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0x88);  // Calibration data start address
  Wire.endTransmission();
  Wire.requestFrom(BMP280_ADDRESS, 24);

  if (Wire.available() >= 24) {
    // Temperature calibration
    bmp280_calib.T1 = (Wire.read() | (Wire.read() << 8));
    bmp280_calib.T2 = Wire.read() | (Wire.read() << 8);
    bmp280_calib.T3 = Wire.read() | (Wire.read() << 8);

    // Pressure calibration
    bmp280_calib.P1 = (Wire.read() | (Wire.read() << 8));
    bmp280_calib.P2 = Wire.read() | (Wire.read() << 8);
    bmp280_calib.P3 = Wire.read() | (Wire.read() << 8);
    bmp280_calib.P4 = Wire.read() | (Wire.read() << 8);
    bmp280_calib.P5 = Wire.read() | (Wire.read() << 8);
    bmp280_calib.P6 = Wire.read() | (Wire.read() << 8);
    bmp280_calib.P7 = Wire.read() | (Wire.read() << 8);
    bmp280_calib.P8 = Wire.read() | (Wire.read() << 8);
    bmp280_calib.P9 = Wire.read() | (Wire.read() << 8);

    Serial.println("BMP280 calibration data loaded");
  }

  // Configure BMP280: normal mode, oversampling 2x temp, 16x pressure
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF4);  // ctrl_meas register
  Wire.write(0x57);  // osrs_t=2x, osrs_p=16x, mode=normal
  Wire.endTransmission();

  // Set filter coefficient
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF5);  // config register
  Wire.write(0xA0);  // filter=16, standby=1s
  Wire.endTransmission();

  delay(100);
}

void setup_rc_receiver() {
  // Initialize CRSF RC receiver on UART2 (PA2/PA3)
  rc_receiver.begin();
  delay(100);
  Serial.println("RC Receiver UART2 initialized at 420k baud");
}

// ============================================================================
// Sensor Reading Functions
// ============================================================================

void read_mpu6050() {
  // Read 14 bytes from MPU-6050: accel (6 bytes) + temp (2 bytes) + gyro (6 bytes)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H register
  Wire.endTransmission();

  Wire.requestFrom(0x68, 14);

  int16_t accel_x_raw = 0, accel_y_raw = 0, accel_z_raw = 0;
  int16_t gyro_x_raw = 0, gyro_y_raw = 0, gyro_z_raw = 0;

  if (Wire.available() >= 14) {
    // Accelerometer
    accel_x_raw = (Wire.read() << 8) | Wire.read();
    accel_y_raw = (Wire.read() << 8) | Wire.read();
    accel_z_raw = (Wire.read() << 8) | Wire.read();

    // Temperature (skip)
    Wire.read();
    Wire.read();

    // Gyroscope
    gyro_x_raw = (Wire.read() << 8) | Wire.read();
    gyro_y_raw = (Wire.read() << 8) | Wire.read();
    gyro_z_raw = (Wire.read() << 8) | Wire.read();

    // Convert to physical units
    // MPU-6050 with ±2g range: 16384 LSB/g
    // MPU-6050 with ±250 deg/s range: 131 LSB/(deg/s) = 7505.7 LSB/(rad/s)
    const float ACCEL_SCALE = 9.81f / 16384.0f;      // m/s^2 per LSB
    const float GYRO_SCALE = (M_PI / 180.0f) / 131.0f;  // rad/s per LSB

    imu_data.accel_x = accel_x_raw * ACCEL_SCALE;
    imu_data.accel_y = accel_y_raw * ACCEL_SCALE;
    imu_data.accel_z = accel_z_raw * ACCEL_SCALE;

    imu_data.gyro_x = gyro_x_raw * GYRO_SCALE;
    imu_data.gyro_y = gyro_y_raw * GYRO_SCALE;
    imu_data.gyro_z = gyro_z_raw * GYRO_SCALE;
  }
}

// ============================================================================
// BMP280 Barometer Reading
// ============================================================================

void read_bmp280() {
  // Read pressure and temperature data from BMP280
  // Registers: 0xF7 (pressure MSB), 0xF8 (pressure LSB), 0xF9 (pressure XLSB)
  //            0xFA (temperature MSB), 0xFB (temperature LSB), 0xFC (temperature XLSB)

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF7);  // Start at pressure data register
  Wire.endTransmission();

  Wire.requestFrom(BMP280_ADDRESS, 6);

  int32_t pressure_raw = 0;
  int32_t temperature_raw = 0;

  if (Wire.available() >= 6) {
    // Read 20-bit pressure value
    pressure_raw = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);

    // Read 20-bit temperature value
    temperature_raw = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);

    // Apply Bosch compensation formulas (simplified)
    // Full implementation would use integer math with lookup tables
    // For now, store raw pressure directly (complementary filter will use it)

    imu_data.pressure = (float)pressure_raw;
  } else {
    // If read fails, use previous value or zero
    imu_data.pressure = 101325.0f;  // Sea level fallback
  }
}

// ============================================================================
// Filter Update
// ============================================================================

void update_filter() {
  filter.update(imu_data, loop_dt);
}

// ============================================================================
// RC Receiver Update
// ============================================================================

void update_rc_receiver() {
  rc_receiver.update();
}

// ============================================================================
// Attitude Controller Update
// ============================================================================

void update_attitude_controller() {
  // Map RC stick inputs to desired attitude angles
  // RC channels are 1000-2000 μs (center = 1500)
  // Scale to ±45 degrees (±0.785 radians) for roll/pitch
  // Scale to ±180 degrees (±π radians) for yaw rate

  // Roll: (rc_roll - 1500) / 10.0 = ±50 degrees max
  // Pitch: (rc_pitch - 1500) / 10.0 = ±50 degrees max
  // Yaw: (rc_yaw - 1500) / 10.0 = ±50 degrees max (converted to rate)

  const float RC_MAX_ANGLE = 50.0f * M_PI / 180.0f;  // 50 degrees to radians
  const float RC_SCALE = RC_MAX_ANGLE / 500.0f;      // Normalize from ±500 (rc - 1500)

  float roll_desired = (rc_channels.roll - 1500.0f) * RC_SCALE;
  float pitch_desired = (rc_channels.pitch - 1500.0f) * RC_SCALE;
  float yaw_desired = (rc_channels.yaw - 1500.0f) * RC_SCALE;

  // Clamp desired angles to safe limits
  roll_desired = constrain(roll_desired, -RC_MAX_ANGLE, RC_MAX_ANGLE);
  pitch_desired = constrain(pitch_desired, -RC_MAX_ANGLE, RC_MAX_ANGLE);

  // Update attitude PID controller
  // Input: desired angles vs actual angles from complementary filter
  AttitudePIDController::Output attitude_output = attitude_pid.update(
    roll_desired, pitch_desired, yaw_desired,
    current_angles.roll, current_angles.pitch, current_angles.yaw,
    loop_dt
  );

  // The attitude controller outputs desired angular rates
  // Feed these to the rate controller
  // The rate controller will output correction values for ESC mixing

  // Get actual angular rates from gyro
  float actual_roll_rate = imu_data.gyro_x;
  float actual_pitch_rate = imu_data.gyro_y;
  float actual_yaw_rate = imu_data.gyro_z;

  // Update rate PID controller with desired vs actual rates
  RatePIDController::RateCorrection rate_output = rate_pid.update(
    attitude_output.roll_rate, actual_roll_rate,
    attitude_output.pitch_rate, actual_pitch_rate,
    attitude_output.yaw_rate, actual_yaw_rate,
    loop_dt
  );

  // At this point, rate_output contains corrections for each axis
  // In a future phase, these will be converted to motor PWM commands via motor mixing
  // For now, we're just computing the control signals in the control loop

  // Debug: the rate corrections could be logged or used for motor mixing
  // (Motor mixing implementation is Task 7)
}

// ============================================================================
// Altitude Controller Update
// ============================================================================

void update_altitude_controller() {
  // Check if throttle is LOW (below 1050 μs) to enable altitude hold
  // Throttle range: 1000-2000 μs (center/hover at ~1500)
  // LOW = throttle stick at bottom = alt-hold mode

  const uint16_t ALT_HOLD_THRESHOLD = 1050;  // Throttle threshold for alt-hold

  // If throttle is low, we're in altitude-hold mode
  if (rc_channels.throttle < ALT_HOLD_THRESHOLD) {
    // Lock the hover altitude if not already set
    // This happens on the first frame when throttle goes low
    if (rc_channels.throttle < ALT_HOLD_THRESHOLD &&
        rc_channels.throttle > 1000) {  // Valid throttle range
      // Only lock altitude once, not every frame
      static bool alt_hold_active = false;
      if (!alt_hold_active) {
        hover_altitude = filter.getAltitude();
        alt_hold_active = true;
        altitude_ctrl.reset();
      }
    }

    // Update altitude controller and get throttle adjustment
    float actual_altitude = filter.getAltitude();
    float throttle_adjustment = altitude_ctrl.update(
      hover_altitude,
      actual_altitude,
      loop_dt
    );

    // In a real implementation, apply throttle_adjustment to the base throttle
    // For now, this is computed for motor mixing integration (Task 7)
    // throttle_output = base_throttle + throttle_adjustment

  } else {
    // Throttle is above threshold: disable altitude hold
    // Pilot has direct throttle control
    static bool alt_hold_active = false;
    alt_hold_active = false;
    altitude_ctrl.reset();
  }
}

// ============================================================================
// Debug Output
// ============================================================================

void print_debug_info() {
  Serial.print("Roll: ");
  Serial.print(current_angles.roll * 180.0f / M_PI, 2);
  Serial.print("° | Pitch: ");
  Serial.print(current_angles.pitch * 180.0f / M_PI, 2);
  Serial.print("° | Yaw: ");
  Serial.print(current_angles.yaw * 180.0f / M_PI, 2);
  Serial.print("° | Alt: ");
  Serial.print(filter.getAltitude(), 1);
  Serial.print("m");

  // Show altitude hold status
  if (rc_channels.throttle < 1050) {
    Serial.print(" | AltHold: ");
    Serial.print(hover_altitude, 1);
    Serial.print("m (Int: ");
    Serial.print(altitude_ctrl.getIntegral(), 3);
    Serial.print(")");
  }
  Serial.println();
}

// ============================================================================
// RC Information Output
// ============================================================================

void print_rc_info() {
  Serial.print("RC: ");

  if (rc_receiver.isConnected()) {
    Serial.print("THR=");
    Serial.print(rc_channels.throttle);
    Serial.print(" ROLL=");
    Serial.print(rc_channels.roll);
    Serial.print(" PITCH=");
    Serial.print(rc_channels.pitch);
    Serial.print(" YAW=");
    Serial.print(rc_channels.yaw);
    Serial.print(" MODE=");
    Serial.print(rc_channels.mode);
    Serial.print(" AUX=");
    Serial.print(rc_channels.aux);
    Serial.println(" [OK]");
  } else {
    Serial.println("SIGNAL LOST");
  }
}
