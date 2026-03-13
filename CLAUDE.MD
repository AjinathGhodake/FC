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

Module: HMC5883L (GY-271)

Purpose:
- Heading detection
- Navigation correction
- Used along with GPS for orientation

Interface:
I2C

Typical connection:
SCL → PB6
SDA → PB7

Status: Purchased

---

## GPS Module

Module: Ublox NEO-6M / GY-NEO6MV2

Purpose:
- Position tracking
- Waypoint navigation
- Return-to-home capability

Interface:
UART

Typical connection:
TX → STM32 RX
RX → STM32 TX

Status: Purchased

---

# Sensors Not Purchased Yet

These sensors are planned for later stages of the drone development.

---

## IMU (Accelerometer + Gyroscope)

Purpose:
- Detect drone orientation
- Measure acceleration
- Stabilization of drone flight

Typical sensors:
MPU6050
MPU9250
ICM20948

Why needed:
A drone **cannot stabilize without an IMU**.

Status: Not purchased yet

Priority: HIGH

---

## Barometer (Altitude Sensor)

Purpose:
- Measure altitude
- Improve flight stability
- Used for altitude hold

Example modules:
BMP280
MS5611

Status: Not purchased yet

Priority: Medium

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
- HMC5883L compass module
- GPS NEO-6M module

Not Yet Purchased:
- IMU sensor
- Barometer
- Telemetry radio
- Jetson Orin Nano
- Camera module
- FPV system
- LiDAR
- Battery