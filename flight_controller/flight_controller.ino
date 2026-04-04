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
#include "gps.h"
#include "pid_controller.h"
#include "motor_mixer.h"
#include "pwm_output.h"
#include "config.h"
#include "mavlink_telemetry.h"

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

// QMC5883P magnetometer I2C address (GY-271 module)
// Note: QMC5883P uses 0x2C (different from QMC5883L=0x0D, HMC5883L=0x1E)
#define QMC5883P_ADDRESS 0x2C

// ============================================================================
// Global Variables
// ============================================================================

ComplementaryFilter filter(0.98f);  // 98% gyro, 2% accel
IMUData imu_data;
Angles current_angles;

uint8_t mpu_addr = 0x68;  // MPU-6050 address (0x68 if AD0=GND, 0x69 if AD0=VCC)

CRSFReceiver rc_receiver;  // CRSF RC receiver
RCChannels rc_channels;    // Current RC channel values

NEO6M gps;                 // GPS receiver (NEO-6M on UART1)
GPSData gps_data;          // Current GPS data

#ifdef MAVLINK_ENABLED
MAVLinkTelemetry mav;      // MAVLink telemetry for QGroundControl
#endif

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
void setup_compass();
void read_mpu6050();
void read_bmp280();
void read_compass();
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
  setup_compass();  // QMC5883P with proper continuous mode initialization
  setup_rc_receiver();
  setup_pwm_output();
  setup_pid_controllers();

  // Initialize GPS on UART1 (PA9=TX, PA10=RX)
  gps.begin();
  Serial.println("GPS NEO-6M UART1 initialized at 9600 baud");

#ifdef MAVLINK_ENABLED
  mav.begin();
  Serial.println("MAVLink telemetry enabled — connect QGroundControl to this COM port");
#endif

  Serial.println("Initialization complete.");
#ifdef USE_MADGWICK
  Serial.println("Sensor Fusion: Madgwick filter (gyro + accel + mag → drift-free yaw)");
#else
  Serial.println("Sensor Fusion: Complementary filter (98% gyro + 2% accel)");
#endif
  Serial.println("RC Receiver (CRSF/ELRS) initialized on UART2");
  Serial.println("GPS NEO-6M on UART1 (PA10=RX, PA9=TX)");
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
  read_compass();  // QMC5883P compass

  // Update GPS (reads available UART bytes, parses NMEA)
  gps.update();
  gps_data = gps.getData();

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

#ifdef MAVLINK_ENABLED
  // Send MAVLink telemetry to QGroundControl
  mav.update(
    current_angles,
    imu_data,
    gps_data,
    armed,
    filter.getAltitude(),
    rc_channels.throttle
  );
#else
  // Print human-readable dashboard to Serial Monitor
  static int loop_counter = 0;
  if (++loop_counter >= DEBUG_LOG_INTERVAL) {
    print_bench_test_log();
    loop_counter = 0;
  }
#endif

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

    // Apply calibration offsets (measured during stationary calibration run)
    accel_x_raw -= 535;
    accel_y_raw -= 415;
    accel_z_raw -= (-706);
    gyro_x_raw  -= 179;
    gyro_y_raw  -= 765;
    gyro_z_raw  -= (-150);

    // Convert to physical units
    // MPU-6050 ±2g: 16384 LSB/g  |  ±250 deg/s: 131 LSB/(deg/s)
    const float ACCEL_SCALE = 9.81f / 16384.0f;
    const float GYRO_SCALE  = (M_PI / 180.0f) / 131.0f;

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
// QMC5883P Compass (Magnetometer) Setup
// ============================================================================

void setup_compass() {
  // Initialize QMC5883P magnetometer at address 0x2C
  // QMC5883P is a DIFFERENT chip from QMC5883L — different register map!
  // Chip ID register 0x00 = 0x80, control register = 0x0A (not 0x09)
  // Data registers start at 0x01 (not 0x00), STATUS is at 0x09

  delay(100);

  Serial.print("QMC5883P: ");

  // Step 1: Verify chip ID (register 0x00 should be 0x80)
  Wire.beginTransmission(QMC5883P_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDRESS, (uint8_t)1);
  uint8_t chip_id = 0;
  if (Wire.available()) {
    chip_id = Wire.read();
  }
  Serial.print("ID=0x");
  Serial.print(chip_id, HEX);

  // Step 2: Write hidden register 0x0D (required for data path)
  Wire.beginTransmission(QMC5883P_ADDRESS);
  Wire.write(0x0D);
  Wire.write(0x40);
  uint8_t e1 = Wire.endTransmission();
  delay(10);

  // Step 3: Write XYZ sign/unlock register 0x29
  Wire.beginTransmission(QMC5883P_ADDRESS);
  Wire.write(0x29);
  Wire.write(0x06);
  uint8_t e2 = Wire.endTransmission();
  delay(10);

  // Step 4: CONF2 register (0x0B) — Range and set/reset (write BEFORE CONF1)
  // Bit 7: SOFT_RST (0 = no reset)
  // Bit 3-2: Range - 00=30G, 01=12G, 10=8G, 11=2G
  // Bit 1-0: Set/Reset - 00=On, 01=Set only, 10=Off
  Wire.beginTransmission(QMC5883P_ADDRESS);
  Wire.write(0x0B);
  Wire.write(0x08);  // 8G range, set/reset ON
  uint8_t e3 = Wire.endTransmission();
  delay(10);

  // Step 5: CONF1 register (0x0A) — THE KEY REGISTER for mode control
  // Bit 7-6: DSR (Downsample Ratio) - 00=1x, 01=2x, 10=4x, 11=8x
  // Bit 5-4: OSR (Oversample Ratio) - 00=8x, 01=4x, 10=2x, 11=1x
  // Bit 3-2: ODR (Output Data Rate) - 00=10Hz, 01=50Hz, 10=100Hz, 11=200Hz
  // Bit 1-0: MODE - 00=Suspend, 01=Normal, 10=Single, 11=Continuous
  //
  // 0x0F = 0000 1111 = DSR 1x + OSR 8x + ODR 200Hz + Continuous mode
  Wire.beginTransmission(QMC5883P_ADDRESS);
  Wire.write(0x0A);
  Wire.write(0x0F);  // Continuous mode, 200Hz, 8x oversample
  uint8_t ctrl_err = Wire.endTransmission();
  delay(50);

  Serial.print(" W[0x0D]=");
  Serial.print(e1);
  Serial.print(" W[0x29]=");
  Serial.print(e2);
  Serial.print(" W[0x0B]=");
  Serial.print(e3);
  Serial.print(" W[0x0A]=");
  Serial.print(ctrl_err);

  // Verify: read back CONF1 (0x0A) and STATUS (0x09)
  Wire.beginTransmission(QMC5883P_ADDRESS);
  Wire.write(0x0A);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDRESS, (uint8_t)1);
  uint8_t conf1_verify = 0;
  if (Wire.available()) {
    conf1_verify = Wire.read();
  }

  Serial.print(" CONF1=0x");
  Serial.print(conf1_verify, HEX);
  Serial.print(" err=");
  Serial.print(ctrl_err);

  if (conf1_verify == 0x0F) {
    Serial.println(" OK (continuous mode)");
  } else {
    Serial.print(" MISMATCH (expected 0x0F, got 0x");
    Serial.print(conf1_verify, HEX);
    Serial.println(")");
  }
}

// ============================================================================
// QMC5883P Compass Reading
// ============================================================================

void read_compass() {
  // QMC5883P register map:
  //   0x00 = Chip ID (0x80)
  //   0x01-0x06 = X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
  //   0x09 = STATUS (bit 0 = DRDY)

  // Check STATUS register (0x09) for data ready
  Wire.beginTransmission(QMC5883P_ADDRESS);
  Wire.write(0x09);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDRESS, (uint8_t)1);

  uint8_t status = 0;
  if (Wire.available()) {
    status = Wire.read();
  }

  // Debug counter
  static int compass_debug_count = 0;
  compass_debug_count++;

  if (!(status & 0x01)) {
    // Data not ready, skip this read
    return;
  }

  // Read 6 bytes starting from register 0x01 (X_LSB)
  Wire.beginTransmission(QMC5883P_ADDRESS);
  Wire.write(0x01);  // Data starts at 0x01 for QMC5883P
  uint8_t err = Wire.endTransmission(false);

  uint8_t n = Wire.requestFrom(QMC5883P_ADDRESS, (uint8_t)6);

  if (err == 0 && n >= 6 && Wire.available() >= 6) {
    // Read raw magnetometer values (little-endian: LSB first)
    int16_t mag_x_raw = Wire.read() | ((int16_t)Wire.read() << 8);
    int16_t mag_y_raw = Wire.read() | ((int16_t)Wire.read() << 8);
    int16_t mag_z_raw = Wire.read() | ((int16_t)Wire.read() << 8);

    // Scale to Gauss (QMC5883P at 8G range: ~3000 LSB/Gauss)
    const float MAG_SCALE = 1.0f / 3000.0f;

    imu_data.mag_x = mag_x_raw * MAG_SCALE;
    imu_data.mag_y = mag_y_raw * MAG_SCALE;
    imu_data.mag_z = mag_z_raw * MAG_SCALE;

    // Calculate heading from magnetic field
    float heading_rad = atan2(imu_data.mag_y, imu_data.mag_x);
    imu_data.heading = heading_rad * 180.0f / M_PI;
    if (imu_data.heading < 0) {
      imu_data.heading += 360.0f;
    }

  } else {
    // Compass read failed — silently skip
  }
}

// ============================================================================
// Filter Update
// ============================================================================

void update_filter() {
  filter.update(imu_data, &gps_data, loop_dt);
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
  unsigned long current_ms = millis();

  Serial.println("========================================");
  Serial.print("T: ");
  Serial.print(current_ms / 1000.0f, 1);
  Serial.print("s | ");
  Serial.println(armed ? "[ARMED]" : "[DISARMED]");

  // IMU (MPU-6050)
  Serial.print("  IMU     | Roll: ");
  Serial.print(current_angles.roll * 180.0f / M_PI, 1);
  Serial.print("°  Pitch: ");
  Serial.print(current_angles.pitch * 180.0f / M_PI, 1);
  Serial.print("°  Yaw: ");
  Serial.print(current_angles.yaw * 180.0f / M_PI, 1);
  Serial.println("°");

  // Barometer (BMP280)
  Serial.print("  BARO    | Alt: ");
  Serial.print(filter.getAltitude(), 1);
  Serial.print("m  Pressure: ");
  Serial.print(imu_data.pressure / 100.0f, 1);
  Serial.println("hPa");

  // Compass (QMC5883P)
  Serial.print("  COMPASS | Hdg: ");
  Serial.print(imu_data.heading, 1);
  Serial.print("°  X: ");
  Serial.print(imu_data.mag_x, 2);
  Serial.print("  Y: ");
  Serial.print(imu_data.mag_y, 2);
  Serial.print("  Z: ");
  Serial.println(imu_data.mag_z, 2);

  // GPS (NEO-6M)
  Serial.print("  GPS     | ");
  if (gps_data.fix_valid) {
    Serial.print("FIX_OK  Sat: ");
    Serial.print(gps_data.satellites);
    Serial.print("  Lat: ");
    Serial.print(gps_data.latitude, 6);
    Serial.print("  Lon: ");
    Serial.print(gps_data.longitude, 6);
    Serial.print("  Alt: ");
    Serial.print(gps_data.altitude_gps, 1);
    Serial.print("m  Speed: ");
    Serial.print(gps_data.speed_knots, 1);
    Serial.println("kts");
  } else {
    Serial.print("NO_FIX  Sat: ");
    Serial.println(gps_data.satellites);
  }

  // RC Receiver
  Serial.print("  RC      | Thr: ");
  Serial.print(rc_channels.throttle);
  Serial.print("  Roll: ");
  Serial.print(rc_channels.roll);
  Serial.print("  Pitch: ");
  Serial.print(rc_channels.pitch);
  Serial.print("  Yaw: ");
  Serial.print(rc_channels.yaw);
  Serial.println(rc_receiver.isConnected() ? "  [CONNECTED]" : "  [NO_SIGNAL]");

  // Motors
  Serial.print("  MOTORS  | M1: ");
  Serial.print(pwm_output.getMotor(1));
  Serial.print("  M2: ");
  Serial.print(pwm_output.getMotor(2));
  Serial.print("  M3: ");
  Serial.print(pwm_output.getMotor(3));
  Serial.print("  M4: ");
  Serial.println(pwm_output.getMotor(4));
}
