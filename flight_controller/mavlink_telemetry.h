#ifndef MAVLINK_TELEMETRY_H
#define MAVLINK_TELEMETRY_H

#include <Arduino.h>
#include "sensors.h"
#include "gps.h"
#include "config.h"

// ============================================================================
// MAVLink Telemetry — QGroundControl integration over USB Serial
//
// Enable by adding #define MAVLINK_ENABLED in config.h
// Requires: Install "MAVLink" library from Arduino Library Manager
//
// How to connect QGC:
//   1. Enable MAVLINK_ENABLED in config.h and flash
//   2. Open QGroundControl
//   3. Application Settings → Comm Links → Add → Serial
//   4. Select your STM32 COM port, baud rate 115200
//   5. Connect — you will see attitude, GPS, altitude live
// ============================================================================

#ifdef MAVLINK_ENABLED
#include <MAVLink.h>

// Vehicle IDs
#define MAV_SYS_ID   1    // System ID (our drone)
#define MAV_COMP_ID  1    // Component ID (autopilot)

class MAVLinkTelemetry {
public:
  MAVLinkTelemetry();

  // Call once in setup() — after Serial.begin()
  void begin();

  // Call every loop iteration — handles rate limiting internally
  void update(
    const Angles&    angles,      // roll/pitch/yaw in radians
    const IMUData&   imu,         // gyro rates + pressure
    const GPSData&   gps,         // GPS fix data
    bool             armed,       // arm state
    float            baro_alt,    // barometer altitude in meters
    uint16_t         throttle_us  // throttle channel (1000-2000 us)
  );

private:
  unsigned long t_heartbeat;
  unsigned long t_attitude;
  unsigned long t_gps;
  unsigned long t_vfr;
  unsigned long t_pressure;

  void send_heartbeat(bool armed);
  void send_attitude(const Angles& a, const IMUData& imu);
  void send_gps_raw(const GPSData& gps);
  void send_global_position(const GPSData& gps, float baro_alt);
  void send_vfr_hud(const GPSData& gps, float heading_deg, float alt, uint16_t throttle_us);
  void send_scaled_pressure(float pressure_pa);

  void write_msg(mavlink_message_t& msg);
};

#endif  // MAVLINK_ENABLED
#endif  // MAVLINK_TELEMETRY_H
