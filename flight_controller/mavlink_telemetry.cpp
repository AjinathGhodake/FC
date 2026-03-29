#include "mavlink_telemetry.h"

#ifdef MAVLINK_ENABLED

#include <math.h>

// ============================================================================
// Constructor
// ============================================================================

MAVLinkTelemetry::MAVLinkTelemetry()
  : t_heartbeat(0), t_attitude(0), t_gps(0), t_vfr(0), t_pressure(0) {}

void MAVLinkTelemetry::begin() {
  // Serial already initialized in setup() — nothing extra needed
  // Timing vars initialized to 0 in constructor so first send fires immediately
}

// ============================================================================
// Main Update — call every loop, handles rate limiting internally
// ============================================================================

void MAVLinkTelemetry::update(
  const Angles&  angles,
  const IMUData& imu,
  const GPSData& gps,
  bool           armed,
  float          baro_alt,
  uint16_t       throttle_us)
{
  unsigned long now = millis();

  // HEARTBEAT at 1 Hz — required or QGC shows "Communication Lost"
  if (now - t_heartbeat >= 1000) {
    send_heartbeat(armed);
    t_heartbeat = now;
  }

  // ATTITUDE at 10 Hz — roll/pitch/yaw live in QGC HUD
  if (now - t_attitude >= 100) {
    send_attitude(angles, imu);
    t_attitude = now;
  }

  // GPS at 5 Hz — position on QGC map, satellite count
  if (now - t_gps >= 200) {
    send_gps_raw(gps);
    if (gps.fix_valid) {
      send_global_position(gps, baro_alt);
    }
    t_gps = now;
  }

  // VFR HUD at 5 Hz — speed, heading, altitude on HUD overlay
  if (now - t_vfr >= 200) {
    send_vfr_hud(gps, imu.heading, baro_alt, throttle_us);
    t_vfr = now;
  }

  // SCALED_PRESSURE at 5 Hz — barometer data
  if (now - t_pressure >= 200) {
    send_scaled_pressure(imu.pressure);
    t_pressure = now;
  }
}

// ============================================================================
// HEARTBEAT — vehicle type + arm state
// ============================================================================

void MAVLinkTelemetry::send_heartbeat(bool armed) {
  mavlink_message_t msg;

  uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  if (armed) base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

  mavlink_msg_heartbeat_pack(
    MAV_SYS_ID, MAV_COMP_ID, &msg,
    MAV_TYPE_QUADROTOR,
    MAV_AUTOPILOT_GENERIC,
    base_mode,
    0,  // custom_mode
    armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY
  );
  write_msg(msg);
}

// ============================================================================
// ATTITUDE — roll, pitch, yaw (radians) + gyro rates
// ============================================================================

void MAVLinkTelemetry::send_attitude(const Angles& a, const IMUData& imu) {
  mavlink_message_t msg;

  mavlink_msg_attitude_pack(
    MAV_SYS_ID, MAV_COMP_ID, &msg,
    millis(),
    a.roll,      // radians (Angles struct is already in radians)
    a.pitch,     // radians
    a.yaw,       // radians
    imu.gyro_x,  // rad/s
    imu.gyro_y,  // rad/s
    imu.gyro_z   // rad/s
  );
  write_msg(msg);
}

// ============================================================================
// GPS_RAW_INT — fix type, position, satellite count
// ============================================================================

void MAVLinkTelemetry::send_gps_raw(const GPSData& gps) {
  mavlink_message_t msg;

  // MAVLink GPS fix types: 0=no GPS, 1=no fix, 2=2D, 3=3D, 4=DGPS
  uint8_t fix_type = 1;  // default = no fix
  if      (gps.fix_quality == 2) fix_type = 4;  // DGPS
  else if (gps.fix_quality == 1) fix_type = 3;  // 3D GPS fix

  // Speed: knots → cm/s
  uint16_t vel = gps.fix_valid ? (uint16_t)(gps.speed_knots * 51.444f) : 0;

  // Course: degrees → centidegrees
  uint16_t cog = gps.fix_valid ? (uint16_t)(gps.course * 100.0f) : 0;

  mavlink_msg_gps_raw_int_pack(
    MAV_SYS_ID, MAV_COMP_ID, &msg,
    (uint64_t)millis() * 1000ULL,        // time_usec
    fix_type,
    (int32_t)(gps.latitude  * 1e7),      // lat degE7
    (int32_t)(gps.longitude * 1e7),      // lon degE7
    (int32_t)(gps.altitude_gps * 1000),  // alt mm MSL
    gps.fix_valid ? 250  : UINT16_MAX,   // eph (cm) — 2.5m when fixed
    UINT16_MAX,                           // epv unknown
    vel,
    cog,
    gps.satellites
  );
  write_msg(msg);
}

// ============================================================================
// GLOBAL_POSITION_INT — lat/lon/alt with velocity vector
// ============================================================================

void MAVLinkTelemetry::send_global_position(const GPSData& gps, float baro_alt) {
  mavlink_message_t msg;

  // Decompose ground speed into vx/vy (cm/s) using course angle
  float speed_ms   = gps.speed_knots * 0.514444f;
  float course_rad = gps.course      * 0.0174533f;
  int16_t vx = (int16_t)(speed_ms * cosf(course_rad) * 100.0f);
  int16_t vy = (int16_t)(speed_ms * sinf(course_rad) * 100.0f);

  mavlink_msg_global_position_int_pack(
    MAV_SYS_ID, MAV_COMP_ID, &msg,
    millis(),
    (int32_t)(gps.latitude  * 1e7),      // lat degE7
    (int32_t)(gps.longitude * 1e7),      // lon degE7
    (int32_t)(gps.altitude_gps * 1000),  // alt MSL mm
    (int32_t)(baro_alt         * 1000),  // relative alt mm (baro)
    vx,
    vy,
    0,                                    // vz — no vertical GPS
    (uint16_t)(gps.course * 100)          // heading cdeg
  );
  write_msg(msg);
}

// ============================================================================
// VFR_HUD — speed, heading, altitude, throttle on QGC HUD overlay
// ============================================================================

void MAVLinkTelemetry::send_vfr_hud(const GPSData& gps, float heading_deg, float alt, uint16_t throttle_us) {
  mavlink_message_t msg;

  float groundspeed = gps.fix_valid ? (gps.speed_knots * 0.514444f) : 0.0f;

  // Convert 1000–2000 µs PWM → 0–100% throttle
  uint16_t throttle_pct = 0;
  if (throttle_us > 1000) {
    throttle_pct = (throttle_us - 1000) / 10;
    if (throttle_pct > 100) throttle_pct = 100;
  }

  mavlink_msg_vfr_hud_pack(
    MAV_SYS_ID, MAV_COMP_ID, &msg,
    0.0f,           // airspeed (no pitot tube)
    groundspeed,    // groundspeed m/s
    (int16_t)heading_deg,
    throttle_pct,
    alt,            // barometer altitude m
    0.0f            // climb rate m/s (future: derive from baro)
  );
  write_msg(msg);
}

// ============================================================================
// SCALED_PRESSURE — barometer absolute pressure
// ============================================================================

void MAVLinkTelemetry::send_scaled_pressure(float pressure_pa) {
  mavlink_message_t msg;

  mavlink_msg_scaled_pressure_pack(
    MAV_SYS_ID, MAV_COMP_ID, &msg,
    millis(),
    pressure_pa / 100.0f,  // Pa → hPa
    0.0f,                   // differential pressure (none)
    2500                    // temperature in cdegC (25.00°C placeholder)
  );
  write_msg(msg);
}

// ============================================================================
// Serialize and write one MAVLink message to Serial
// ============================================================================

void MAVLinkTelemetry::write_msg(mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

#endif  // MAVLINK_ENABLED
