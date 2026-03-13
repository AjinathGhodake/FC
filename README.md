# STM32 Flight Controller

Custom flight controller built from scratch on STM32F411CEU6 (Black Pill) for an S500 quadcopter frame.

## Features

- **Sensor Fusion** — Complementary filter (98% gyro + 2% accel) for stable roll/pitch/yaw
- **Altitude Hold** — BMP280 barometer + vertical acceleration fusion
- **RC Input** — ExpressLRS receiver via CRSF protocol (420k baud UART)
- **Dual-Loop PID** — Cascaded attitude (outer) + rate (inner) control
- **Motor Mixing** — X-frame quadcopter mixing matrix
- **Safety** — Arm/disarm via RC switch, automatic failsafe on signal loss

## Hardware

| Component | Module | Interface |
|---|---|---|
| MCU | STM32F411CEU6 (Black Pill) | — |
| IMU | MPU-6050 (accel + gyro) | I2C (PB6/PB7) |
| Barometer | BMP280 | I2C (PB6/PB7) |
| RC Receiver | ExpressLRS RP2 v2 | UART2 (PA2/PA3) |
| Frame | S500 X-frame quadcopter | — |
| ESCs | 40A x4 | PWM 50Hz |
| Battery | 4S 5200mAh LiPo | — |

## Pin Map

| Pin | Function |
|---|---|
| PA0 | Motor 1 — Front Left (PWM) |
| PA1 | Motor 2 — Front Right (PWM) |
| PB0 | Motor 3 — Rear Right (PWM) |
| PB1 | Motor 4 — Rear Left (PWM) |
| PA2 | ELRS TX (UART2) |
| PA3 | ELRS RX (UART2) |
| PA9 | Debug Serial TX (UART1) |
| PA10 | Debug Serial RX (UART1) |
| PB6 | I2C SCL |
| PB7 | I2C SDA |
| PC13 | Onboard LED |

## Architecture

```
Sensors (I2C)              ELRS Receiver (UART2)
    |                              |
readSensors()              readElrsInput()
    |                              |
complementaryFilter()      mapChannels()
    |                              |
[roll, pitch, yaw]         [throttle, roll, pitch, yaw]
    |                              |
    +------------- + --------------+
                   |
            attitudePID()        (outer loop, 50 Hz)
                   |
            ratePID()            (inner loop, 200 Hz)
                   |
            altitudeController()
                   |
            motorMixer()         (X-frame matrix)
                   |
            setPWM() -> ESCs -> Motors
```

## Project Structure

```
stm32_flight_controller/
├── README.md
├── CLAUDE.MD                         # Hardware procurement notes
├── docs/                             # Design specs and plans
├── tests/                            # Unit tests (run on host)
└── flight_controller/                # Arduino sketch
    ├── flight_controller.ino         # Main loop
    ├── sensors.h / sensors.cpp       # Complementary filter
    ├── rc_input.h / rc_input.cpp     # CRSF protocol parser
    ├── pid_controller.h / .cpp       # PID controllers (rate + attitude + altitude)
    ├── motor_mixer.h / .cpp          # X-frame mixing matrix
    ├── pwm_output.h / .cpp           # ESC PWM output (Servo library)
    └── config.h                      # PID gains and tuning parameters
```

## Setup

### Requirements

- Arduino IDE 2.x
- STM32 board support package (STM32duino)
- STM32CubeProgrammer (for DFU/SWD flashing)
- ST-Link V2 (recommended for flashing + debugging)

### Arduino IDE Settings

| Setting | Value |
|---|---|
| Board | Generic STM32F4 series |
| Board part number | BlackPill F411CE |
| Upload method | STM32CubeProgrammer (SWD) |
| USB support | CDC (generic 'Serial' supersede U(S)ART) |

### Flashing

**Via ST-Link (recommended):**
1. Connect ST-Link to SWD pins (SWDIO→PA13, SWCLK→PA14, GND, 3.3V)
2. Select upload method: STM32CubeProgrammer (SWD)
3. Click Upload in Arduino IDE

**Via DFU (USB):**
1. Hold BOOT0 → tap NRST → release BOOT0
2. Select upload method: STM32CubeProgrammer (DFU)
3. Click Upload, then unplug/replug USB to run

## PID Tuning

Default gains in `config.h`:

| Controller | Kp | Ki | Kd |
|---|---|---|---|
| Rate (inner) | 0.015 | 0.02 | 0.0 |
| Attitude (outer) | 4.5 | 0.05 | 0.15 |
| Altitude | 0.3 | 0.05 | 0.1 |

**Tuning tips:**
- Oscillating → reduce Kp
- Sluggish response → increase Kp
- Slow drift → increase Ki
- Overshooting → increase Kd

## Safety

- Motors only spin when **armed** (RC mode switch > 1500 μs)
- Automatic **disarm** on RC signal loss (500ms timeout)
- Always test with **propellers removed** first
- Use a **tether** for first flights with props

## Development Plan

- [x] Phase 1 — Sensor integration (MPU-6050, BMP280)
- [x] Phase 2 — Sensor fusion (complementary filter)
- [x] Phase 3 — RC input (ELRS/CRSF)
- [x] Phase 4 — PID controllers (rate + attitude + altitude)
- [x] Phase 5 — Motor mixing and PWM output
- [ ] Phase 6 — Bench testing and PID tuning
- [ ] Phase 7 — HMC5883L magnetometer (heading)
- [ ] Phase 8 — GPS NEO-6M (position hold, RTH)
- [ ] Phase 9 — First flight

## License

Personal project — not for production use.
