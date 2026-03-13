/*
 * Flight Controller Sketch for STM32F411 (Black Pill)
 *
 * Integrates:
 * - MPU-6050 IMU (accelerometer + gyroscope)
 * - BMP280 Barometer
 * - HMC5883L Magnetometer
 * - GPS NEO-6M
 *
 * Uses complementary filter for sensor fusion (accel + gyro -> roll/pitch/yaw)
 */

#include <Wire.h>
#include "sensors.h"

// ============================================================================
// Configuration
// ============================================================================

#define LOOP_RATE_HZ 100      // Main loop frequency
#define LOOP_TIME_MS (1000.0f / LOOP_RATE_HZ)  // ~10 ms per loop
#define LOOP_TIME_S  (LOOP_TIME_MS / 1000.0f)  // ~0.01 s per loop

// LED pin
#define LED_PIN PC13

// ============================================================================
// Global Variables
// ============================================================================

ComplementaryFilter filter(0.98f);  // 98% gyro, 2% accel
IMUData imu_data;
Angles current_angles;

unsigned long last_loop_time = 0;
float loop_dt = LOOP_TIME_S;

// ============================================================================
// Function Prototypes
// ============================================================================

void setup_imu();
void setup_led();
void read_mpu6050();
void update_filter();
void print_debug_info();

// ============================================================================
// Arduino Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial to stabilize

  Serial.println("Flight Controller Starting...");

  setup_led();
  setup_imu();

  Serial.println("Initialization complete.");
  Serial.println("Complementary Filter: 98% gyro + 2% accel");

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

  // Update complementary filter
  update_filter();

  // Get current angles (debugging)
  current_angles = filter.getAngles();

  // Print debug info every 10 loops (1 second at 100 Hz)
  static int loop_counter = 0;
  if (++loop_counter >= 10) {
    print_debug_info();
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
// Filter Update
// ============================================================================

void update_filter() {
  filter.update(imu_data, loop_dt);
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
  Serial.println("°");
}
