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
#include "rc_input.h"
#include "pid_controller.h"
#include "motor_mixer.h"
#include "pwm_output.h"
#include "config.h"

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

uint8_t mpu_addr = 0x68;  // MPU-6050 address (0x68 if AD0=GND, 0x69 if AD0=VCC)

CRSFReceiver rc_receiver;  // CRSF RC receiver
RCChannels rc_channels;    // Current RC channel values

AttitudePIDController attitude_pid;  // Cascaded attitude controller
RatePIDController rate_pid;          // Inner rate controller
AltitudeController altitude_ctrl;    // Altitude hold controller
MotorMixer motor_mixer;              // X-frame motor mixing
PWMController pwm_output;            // PWM output to ESCs

unsigned long last_loop_time = 0;
float loop_dt = LOOP_TIME_S;

// Arm/disarm state
static bool armed = false;           // Arm/disarm flag
static unsigned long disarm_timeout = 0;  // Timeout counter for disarm detection

// Altitude hold state
static float hover_altitude = 0.0f;  // Locked altitude when in alt-hold mode (meters)
static float throttle_adjustment = 0.0f;  // Altitude trim (from altitude controller)

// Motor control state
static RatePIDController::RateCorrection rate_output = {0.0f, 0.0f, 0.0f};  // Rate PID outputs
static MotorOutput motor_output = {1500, 1500, 1500, 1500};  // Current motor PWM values

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
void setup_pwm_output();
void setup_pid_controllers();
void read_mpu6050();
void read_bmp280();
void update_filter();
void update_rc_receiver();
void update_arm_disarm_logic();
void update_attitude_controller();
void update_altitude_controller();
void update_motor_mixer();
void update_pwm_output();
void print_bench_test_log();

// ============================================================================
// Arduino Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  // Wait up to 5 seconds for serial monitor to open, then proceed anyway
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 5000) { delay(10); }
  delay(200);

  Serial.println("Flight Controller Starting...");

  setup_led();
  setup_imu();
  setup_rc_receiver();
  setup_pwm_output();
  setup_pid_controllers();

  Serial.println("Initialization complete.");
  Serial.println("Complementary Filter: 98% gyro + 2% accel");
  Serial.println("RC Receiver (CRSF/ELRS) initialized on UART2");
  Serial.println("PWM Output (50 Hz) on PA0,PA1,PB0,PB1 (Motors 1-4)");
  Serial.println("SAFETY: Arm with RC Mode > 1500 μs, Disarm with Mode < 1500 μs");

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

  // Get RC channels
  rc_channels = rc_receiver.getChannels();

  // Update arm/disarm logic (check RC Mode switch)
  update_arm_disarm_logic();

  // Update complementary filter
  update_filter();

  // Get current angles (debugging)
  current_angles = filter.getAngles();

  // Update cascaded attitude and rate controllers (only if armed)
  if (armed) {
    update_attitude_controller();
    update_altitude_controller();
    update_motor_mixer();
  }

  // Update PWM output with motor values (or disarm all if not armed)
  update_pwm_output();

  // Print debug info at configurable interval
  static int loop_counter = 0;
  if (++loop_counter >= DEBUG_LOG_INTERVAL) {
    print_bench_test_log();
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

// Reset the STM32 I2C peripheral and reinitialize Wire
static void i2c_reset() {
  Wire.end();
  delay(10);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);
  GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));
  GPIOB->PUPDR |=  ((1 << 12) | (1 << 14));
  delay(10);
}

void setup_imu() {
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);

  // Enable internal pull-ups on PB6/PB7
  GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));
  GPIOB->PUPDR |=  ((1 << 12) | (1 << 14));

  delay(250);  // Wait for sensors to power up

  // --- Initialize MPU-6050 ---
  // No scan - go directly to init to avoid corrupting I2C bus state.
  // Try address 0x68 first (AD0=GND), then 0x69 (AD0=VCC).
  mpu_addr = 0;
  uint8_t try_addrs[2] = {0x68, 0x69};

  for (int i = 0; i < 2; i++) {
    uint8_t addr = try_addrs[i];

    // Reset I2C before each attempt (clears any stuck state)
    i2c_reset();

    // Try reading WHO_AM_I register (0x75) — should return 0x68
    Wire.beginTransmission(addr);
    Wire.write(0x75);
    uint8_t err = Wire.endTransmission(false);
    uint8_t n = Wire.requestFrom(addr, (uint8_t)1);
    uint8_t who = Wire.available() ? Wire.read() : 0xFF;

    Serial.print("MPU probe 0x");
    Serial.print(addr, HEX);
    Serial.print(" -> err=");
    Serial.print(err);
    Serial.print(" who=0x");
    Serial.println(who, HEX);

    if (err == 0 && who == 0x68) {
      mpu_addr = addr;
      break;
    }
  }

  if (mpu_addr == 0) {
    Serial.println("ERROR: MPU-6050 not responding. Check SDA->PB7, SCL->PB6, VCC->3.3V");
    mpu_addr = 0x68;
  } else {
    Serial.print("MPU-6050 OK at 0x");
    Serial.println(mpu_addr, HEX);

    // Wake up MPU-6050
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Serial.print("  Wake: ");
    Serial.println(Wire.endTransmission());

    delay(100);

    // Gyro range ±250 deg/s
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Accel range ±2g
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(50);

    // Verify: test read of 14 bytes
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x3B);
    uint8_t te = Wire.endTransmission(false);
    uint8_t nb = Wire.requestFrom(mpu_addr, (uint8_t)14);
    Serial.print("  Data read: err=");
    Serial.print(te);
    Serial.print(" bytes=");
    Serial.println(nb);
    while (Wire.available()) Wire.read();
  }

  // Reset I2C before BMP280 init to ensure clean state
  i2c_reset();

  // --- Initialize BMP280 ---
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xD0);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDRESS, (uint8_t)1);
  uint8_t bmp_id = Wire.available() ? Wire.read() : 0;
  Serial.print("BMP280 chip ID: 0x");
  Serial.println(bmp_id, HEX);

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDRESS, (uint8_t)24);

  if (Wire.available() >= 24) {
    uint8_t buf[24];
    for (int i = 0; i < 24; i++) buf[i] = Wire.read();

    bmp280_calib.T1 = buf[0]  | (buf[1] << 8);
    bmp280_calib.T2 = buf[2]  | (buf[3] << 8);
    bmp280_calib.T3 = buf[4]  | (buf[5] << 8);
    bmp280_calib.P1 = buf[6]  | (buf[7] << 8);
    bmp280_calib.P2 = buf[8]  | (buf[9] << 8);
    bmp280_calib.P3 = buf[10] | (buf[11] << 8);
    bmp280_calib.P4 = buf[12] | (buf[13] << 8);
    bmp280_calib.P5 = buf[14] | (buf[15] << 8);
    bmp280_calib.P6 = buf[16] | (buf[17] << 8);
    bmp280_calib.P7 = buf[18] | (buf[19] << 8);
    bmp280_calib.P8 = buf[20] | (buf[21] << 8);
    bmp280_calib.P9 = buf[22] | (buf[23] << 8);
    Serial.println("BMP280 calibration data loaded");
  } else {
    Serial.println("BMP280 calib FAILED");
  }

  // Configure BMP280: normal mode, osrs_t=2x, osrs_p=16x, filter=16
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF5);
  Wire.write(0xA0);
  Wire.endTransmission();

  delay(100);
}

void setup_rc_receiver() {
  // Initialize CRSF RC receiver on UART2 (PA2/PA3)
  rc_receiver.begin();
  delay(100);
  Serial.println("RC Receiver UART2 initialized at 420k baud");
}

void setup_pwm_output() {
  // Initialize PWM output controller (Timer 2 on PA0-PA3)
  pwm_output.begin();
  delay(100);
  // Start with all motors disarmed (1000 μs)
  pwm_output.disarmAll();
}

void setup_pid_controllers() {
  // Initialize PID controller gains from config.h

  // Rate PID Controller (inner loop - gyro stabilization)
  rate_pid.setGains(
    RATE_PID_KP, RATE_PID_KI, RATE_PID_KD,  // Roll/Pitch gains
    RATE_PID_KP, RATE_PID_KI, RATE_PID_KD   // Yaw gains
  );

  // Attitude PID Controller (outer loop - angle control)
  attitude_pid.setGains(ATT_PID_KP, ATT_PID_KI, ATT_PID_KD);

  // Altitude Controller
  altitude_ctrl.setGains(ALT_PID_KP, ALT_PID_KI, ALT_PID_KD);

  Serial.println("PID Gains Loaded:");
  Serial.print("  Rate PID: Kp=");
  Serial.print(RATE_PID_KP);
  Serial.print(" Ki=");
  Serial.print(RATE_PID_KI);
  Serial.print(" Kd=");
  Serial.println(RATE_PID_KD);

  Serial.print("  Attitude PID: Kp=");
  Serial.print(ATT_PID_KP);
  Serial.print(" Ki=");
  Serial.print(ATT_PID_KI);
  Serial.print(" Kd=");
  Serial.println(ATT_PID_KD);

  Serial.print("  Altitude PID: Kp=");
  Serial.print(ALT_PID_KP);
  Serial.print(" Ki=");
  Serial.print(ALT_PID_KI);
  Serial.print(" Kd=");
  Serial.println(ALT_PID_KD);
}

// ============================================================================
// Sensor Reading Functions
// ============================================================================

void read_mpu6050() {
  // Read 14 bytes from MPU-6050: accel (6 bytes) + temp (2 bytes) + gyro (6 bytes)
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H register
  Wire.endTransmission(false);  // Repeated start (no STOP before read)

  Wire.requestFrom(mpu_addr, (uint8_t)14);

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
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();

  Wire.requestFrom(BMP280_ADDRESS, 6);

  if (Wire.available() >= 6) {
    int32_t pressure_raw = ((int32_t)Wire.read() << 12) | ((int32_t)Wire.read() << 4) | (Wire.read() >> 4);
    int32_t temperature_raw = ((int32_t)Wire.read() << 12) | ((int32_t)Wire.read() << 4) | (Wire.read() >> 4);

    // Bosch compensation for temperature
    int32_t var1 = ((((temperature_raw >> 3) - ((int32_t)bmp280_calib.T1 << 1))) * ((int32_t)bmp280_calib.T2)) >> 11;
    int32_t var2 = (((((temperature_raw >> 4) - ((int32_t)bmp280_calib.T1)) * ((temperature_raw >> 4) - ((int32_t)bmp280_calib.T1))) >> 12) * ((int32_t)bmp280_calib.T3)) >> 14;
    int32_t t_fine = var1 + var2;

    // Bosch compensation for pressure
    int64_t var1_p = ((int64_t)t_fine) - 128000;
    int64_t var2_p = var1_p * var1_p * (int64_t)bmp280_calib.P6;
    var2_p = var2_p + ((var1_p * (int64_t)bmp280_calib.P5) << 17);
    var2_p = var2_p + (((int64_t)bmp280_calib.P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)bmp280_calib.P3) >> 8) + ((var1_p * (int64_t)bmp280_calib.P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)bmp280_calib.P1) >> 33;

    if (var1_p != 0) {
      int64_t p = 1048576 - pressure_raw;
      p = (((p << 31) - var2_p) * 3125) / var1_p;
      var1_p = (((int64_t)bmp280_calib.P9) * (p >> 13) * (p >> 13)) >> 25;
      var2_p = (((int64_t)bmp280_calib.P8) * p) >> 19;
      p = ((p + var1_p + var2_p) >> 8) + (((int64_t)bmp280_calib.P7) << 4);
      imu_data.pressure = (float)p / 256.0f;  // Pressure in Pa
    }
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
// Arm/Disarm Logic
// ============================================================================

void update_arm_disarm_logic() {
  // Arm/disarm is controlled by RC Mode switch (channel 4, RC ch5 in CRSF)
  // Arm condition: Mode > 1500 μs AND RC signal is connected
  // Disarm condition: Mode < 1500 μs (immediately)
  //
  // Safety: If RC signal is lost, automatically disarm after timeout

  const uint16_t MODE_THRESHOLD = 1500;      // Arm/disarm threshold
  const unsigned long SIGNAL_LOSS_TIMEOUT = 500;  // ms before automatic disarm

  bool signal_connected = rc_receiver.isConnected();
  bool mode_armed = (rc_channels.mode > MODE_THRESHOLD);

  // Check for RC signal loss
  if (!signal_connected) {
    // Signal lost: automatically disarm
    if (armed) {
      armed = false;
      pwm_output.disarmAll();
      Serial.println("DISARM: RC signal lost!");
    }
    return;
  }

  // Signal present: check mode switch
  if (mode_armed && !armed) {
    // Transition from disarmed to armed
    armed = true;
    // Reset integral terms on arm
    attitude_pid.reset();
    rate_pid.reset();
    altitude_ctrl.reset();
    Serial.println("ARMED: Ready for flight!");
  } else if (!mode_armed && armed) {
    // Transition from armed to disarmed
    armed = false;
    pwm_output.disarmAll();
    Serial.println("DISARM: Mode switch down");
  }
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
  rate_output = rate_pid.update(
    attitude_output.roll_rate, actual_roll_rate,
    attitude_output.pitch_rate, actual_pitch_rate,
    attitude_output.yaw_rate, actual_yaw_rate,
    loop_dt
  );

  // At this point, rate_output contains corrections for each axis
  // These are fed to motor mixing in update_motor_mixer() for ESC control
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
    throttle_adjustment = altitude_ctrl.update(
      hover_altitude,
      actual_altitude,
      loop_dt
    );

    // throttle_adjustment ranges from -100 to +100
    // This will be applied to the throttle in motor_mixer

  } else {
    // Throttle is above threshold: disable altitude hold
    // Pilot has direct throttle control
    static bool alt_hold_active = false;
    alt_hold_active = false;
    altitude_ctrl.reset();
  }
}

// ============================================================================
// Motor Mixer Update
// ============================================================================

void update_motor_mixer() {
  // Convert RC throttle (1000-2000 μs) to 0-100%
  // 1000 μs = 0% (no thrust)
  // 1500 μs = 50% (hover)
  // 2000 μs = 100% (max thrust)
  float throttle_percent = (rc_channels.throttle - 1000.0f) / 10.0f;
  throttle_percent = constrain(throttle_percent, 0.0f, 100.0f);

  // Mix motors with:
  // - throttle: from RC stick
  // - roll/pitch/yaw: from rate PID corrections (in -100..+100 range)
  // - altitude_trim: from altitude controller
  motor_output = motor_mixer.mix(
    throttle_percent,
    rate_output.roll * 100.0f / 3.0f,      // Convert rad/s to -100..+100 (-3 rad/s = -100)
    rate_output.pitch * 100.0f / 3.0f,     // Convert rad/s to -100..+100 (-3 rad/s = -100)
    rate_output.yaw * 100.0f / 3.0f,       // Convert rad/s to -100..+100 (-3 rad/s = -100)
    throttle_adjustment
  );

  // At this point, motor_output.m1-m4 contain PWM values (1000-2000 μs)
  // These will be sent to ESCs in update_pwm_output()
}

// ============================================================================
// PWM Output Update
// ============================================================================

void update_pwm_output() {
  // Send motor outputs to PWM controller
  // If armed: send motor_output values to ESCs
  // If disarmed: ESCs are already at 1000 μs (safe state)

  if (armed) {
    // Write motor output values to PWM controller
    pwm_output.setAllMotors(
      motor_output.m1,
      motor_output.m2,
      motor_output.m3,
      motor_output.m4
    );
  }
  // If not armed, PWM is already at 1000 μs from disarmAll()
}

// ============================================================================
// Bench Test Logging
// ============================================================================
// Comprehensive debug logging for bench testing and PID tuning
// Format: "T:1234 | Att: R=0.5 P=1.2 Y=-0.3 | RC: Th=1500 | Motors: 1500 1500 1500 1500"

void print_bench_test_log() {
  // Get current time in milliseconds (for correlation with logs)
  unsigned long current_ms = millis();

  // Print timestamp
  Serial.print("T:");
  Serial.print(current_ms);

  // Print arm status
  Serial.print(" | [");
  Serial.print(armed ? "ARMED" : "DISARM");
  Serial.print("] ");

  // Print attitude angles in degrees
  Serial.print("Att: R=");
  Serial.print(current_angles.roll * 180.0f / M_PI, 1);
  Serial.print(" P=");
  Serial.print(current_angles.pitch * 180.0f / M_PI, 1);
  Serial.print(" Y=");
  Serial.print(current_angles.yaw * 180.0f / M_PI, 1);

  // Print RC inputs
  Serial.print(" | RC: Th=");
  Serial.print(rc_channels.throttle);
  Serial.print(" Ro=");
  Serial.print(rc_channels.roll);
  Serial.print(" Pi=");
  Serial.print(rc_channels.pitch);
  Serial.print(" Ya=");
  Serial.print(rc_channels.yaw);

  // Show RC signal status
  if (!rc_receiver.isConnected()) {
    Serial.print(" [NO_SIGNAL]");
  }

  // Print motor outputs
  Serial.print(" | Motors: ");
  Serial.print(pwm_output.getMotor(1));
  Serial.print(" ");
  Serial.print(pwm_output.getMotor(2));
  Serial.print(" ");
  Serial.print(pwm_output.getMotor(3));
  Serial.print(" ");
  Serial.print(pwm_output.getMotor(4));

  // Print altitude info
  Serial.print(" | Alt: ");
  Serial.print(filter.getAltitude(), 1);
  Serial.print("m");

  // Print rate corrections (from rate PID)
  Serial.print(" | Rate: R=");
  Serial.print(rate_output.roll, 3);
  Serial.print(" P=");
  Serial.print(rate_output.pitch, 3);
  Serial.print(" Y=");
  Serial.print(rate_output.yaw, 3);

  Serial.println();
}
