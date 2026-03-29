# Hardware Procurement Status

This section tracks which components have already been purchased and which are planned for future phases.

---

# Sensors and Modules Purchased

## STM32 Development Board
Board: STM32F411CEU6 (Black Pill)

Purpose:
- Embedded development
- Sensor integration testing
- Flight controller experiments

Status: Purchased

---

## Magnetometer (Compass)

Module: GY-271 board with **QMC5883P** chip (by QST Corporation)

IMPORTANT: The GY-271 module was labeled/sold as HMC5883L but actually contains a QMC5883P chip.
This is a completely different chip with a different register map.
- QMC5883P I2C address: 0x2C
- HMC5883L address: 0x1E (NOT this chip)
- QMC5883L address: 0x0D (NOT this chip either)
- Chip ID: register 0x00 = 0x80

QMC5883P Register Map:
- 0x00: Chip ID (0x80)
- 0x01-0x06: Data registers (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
- 0x09: STATUS register (read-only, bit 0 = DRDY)
- 0x0A: CONF1/CTL1 (mode, ODR, oversample) — control register
- 0x0B: CONF2/CTL2 (range, set/reset, soft reset bit 7)
- 0x0D: Hidden init register (write 0x40)
- 0x29: XYZ sign/unlock register (write 0x06)

CONF1 (0x0A) bits: [7:6]=DSR, [5:4]=OSR, [3:2]=ODR, [1:0]=MODE
MODE: 00=Suspend, 01=Normal, 10=Single, 11=Continuous

Purpose:
- Heading detection
- Navigation correction
- Used along with GPS for orientation

Interface:
I2C (works with STM32 internal pull-ups, external pull-ups NOT needed)

Connection:
SCL → PB6
SDA → PB7

Status: Purchased

---

## GPS Module

Module: GY-NEO6MV2 (u-blox NEO-6M GPS chipset)

Specs:
- Receiver: u-blox NEO-6M
- Sensitivity: -161 dBm (tracking)
- Positioning accuracy: <2500mm (2.5m)
- Flash memory: 24C32 EEPROM (32Kb) for config storage
- Antenna: Built-in ceramic + external active antenna included
- Protocol: NMEA standard output
- Features: Real-time 3D positioning, velocity output, LED indicator

Purpose:
- Position tracking (latitude, longitude)
- Waypoint navigation
- Return-to-home capability
- Speed and altitude data

Interface:
UART (TTL serial), default baud rate 9600

Connection:
GPS TX → STM32 RX
GPS RX → STM32 TX
VCC → 3.3V or 5V
GND → GND

Status: Purchased

---

# Sensors Not Purchased Yet

These sensors are planned for later stages of the drone development.

---

## IMU (Accelerometer + Gyroscope)

Module: MPU-6050

Purpose:
- Detect drone orientation
- Measure acceleration
- Stabilization of drone flight

Interface: I2C (address 0x68)
Connection: SCL → PB6, SDA → PB7

Status: Purchased and working

---

## Barometer (Altitude Sensor)

Module: BMP280

Purpose:
- Measure altitude
- Improve flight stability
- Used for altitude hold

Interface: I2C (address 0x76, CSB→3.3V, SDO→GND)

Status: Purchased and working

---

## Camera Module

Candidate:
Raspberry Pi Camera Module 3

Purpose:
- Computer vision
- Object detection
- Video recording

Connected to:
Jetson Orin Nano

Status: Not purchased yet

Priority: Medium

---

## Telemetry Radio

Purpose:
- Long distance communication
- Send telemetry data to ground station
- Receive commands

Typical range:
1–3 km

Examples:
433 MHz SiK Telemetry
915 MHz Telemetry Radio

Status: Not purchased yet

Priority: HIGH

---

## LiDAR Sensor (Advanced Phase)

Example:
Livox MID-360

Purpose:
- 3D environment mapping
- Obstacle avoidance
- Autonomous navigation

Note:
Very expensive and not required for initial prototype.

Status: Not purchased yet

Priority: Low (Future phase)

---

# Components Planned but Not Yet Purchased

## Companion Computer

Board:
Jetson Orin Nano

Purpose:
- AI processing
- Object detection
- Vision navigation

Status: Not purchased yet

---

## FPV Video System

Purpose:
- Live video transmission
- Manual monitoring

Options:
Analog FPV
Digital FPV

Status: Not purchased yet

---

# Battery System

Battery type:
4S LiPo Battery

Reason:
- Good balance of power
- Lower cost compared to 6S
- Compatible with most ESC systems

Status: Planned (not purchased yet)

---

# Procurement Summary

Purchased Components:
- STM32F411CEU6 development board
- QMC5883P compass module (GY-271, sold as HMC5883L but actually QMC5883P at 0x2C)
- GPS NEO-6M module

Not Yet Purchased:
- Telemetry radio
- Jetson Orin Nano
- Camera module
- FPV system
- LiDAR
- Battery