#include <Arduino.h>
#include "gps.h"
#include "config.h"

// Serial1 is already defined by STM32 core (UART1: PA10=RX, PA9=TX)
// No need to redeclare — just use Serial1.begin() directly

// ============================================================================
// Constructor
// ============================================================================

NEO6M::NEO6M()
  : sentence_index(0), sentence_started(false), field_count(0),
    bytes_received(0), sentences_parsed(0), checksum_errors(0) {
  // Initialize GPS data to defaults
  data.latitude = 0.0f;
  data.longitude = 0.0f;
  data.altitude_gps = 0.0f;
  data.speed_knots = 0.0f;
  data.course = 0.0f;
  data.satellites = 0;
  data.fix_quality = 0;
  data.fix_valid = false;
  data.last_fix_time = 0;
}

// ============================================================================
// Public Methods
// ============================================================================

void NEO6M::begin() {
  // Explicitly set UART1 pins (PA10=RX, PA9=TX) for STM32F411
  Serial1.setRx(PA10);
  Serial1.setTx(PA9);
  Serial1.begin(GPS_BAUDRATE);

  delay(500);  // Wait for module to stabilize

  // Configure NMEA output rate (1Hz)
  // $PMTK220,1000*1F sets update rate to 1Hz
  Serial1.println("$PMTK220,1000*1F");
  delay(100);

  // Enable ONLY GGA and RMC sentences (disable others to reduce clutter)
  // $PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28
  // Format: msg_id,GLL,RMC,VTG,GGA,GSA,GSV,...
  Serial1.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(100);

  // Warm start: reacquire fix using existing almanac
  // $PMTK102*31 = warm start (safe, uses saved almanac)
  Serial1.println("$PMTK102*31");
  delay(500);

  Serial.println("[GPS] NEO-6M initialized (warm start + NMEA configured)");
}

void NEO6M::update() {
  // Check fix timeout
  if (data.fix_valid && (millis() - data.last_fix_time) > GPS_FIX_TIMEOUT) {
    data.fix_valid = false;
  }

  // Diagnostic: disabled for clean output
  // Uncomment below to debug GPS parsing issues
  /*
  static unsigned long last_diagnostic = 0;
  if (millis() - last_diagnostic >= 5000) {
    Serial.print("[GPS_DEBUG] Sats: ");
    Serial.print(data.satellites);
    Serial.print(" | Fix: ");
    Serial.print(data.fix_valid ? "YES" : "NO");
    Serial.print(" | Bytes: ");
    Serial.print(bytes_received);
    Serial.print(" | Sentences: ");
    Serial.println(sentences_parsed);
    last_diagnostic = millis();
  }
  */

  // Read all available bytes from UART1
  while (Serial1.available()) {
    char c = Serial1.read();
    bytes_received++;

    if (c == '$') {
      // Start of new NMEA sentence
      sentence_started = true;
      sentence_index = 0;
      sentence[sentence_index++] = c;
    } else if (sentence_started) {
      if (c == '\r' || c == '\n') {
        // End of sentence
        if (sentence_index > 0) {
          sentence[sentence_index] = '\0';
          process_sentence();
        }
        sentence_started = false;
        sentence_index = 0;
      } else {
        // Accumulate character
        sentence[sentence_index++] = c;

        // Overflow protection
        if (sentence_index >= NMEA_MAX_LEN - 1) {
          sentence_started = false;
          sentence_index = 0;
        }
      }
    }
  }

#ifndef MAVLINK_ENABLED
#ifndef DISABLE_GPS_DEBUG
  // Debug: print clean GPS status every 3 seconds (text mode only)
  static unsigned long last_debug = 0;
  if (millis() - last_debug >= 3000) {
    Serial.print("GPS: ");
    if (data.fix_valid) {
      Serial.print("FIX_OK ");
      Serial.print(data.satellites);
      Serial.print("sat | Lat: ");
      Serial.print(data.latitude, 6);
      Serial.print(" | Lon: ");
      Serial.print(data.longitude, 6);
      Serial.print(" | Alt: ");
      Serial.print(data.altitude_gps, 1);
      Serial.print("m | Speed: ");
      Serial.print(data.speed_knots, 1);
      Serial.println("kts");
    } else {
      Serial.print("SEARCHING ");
      Serial.print(data.satellites);
      Serial.print("sat | bytes=");
      Serial.println(bytes_received);
    }
    last_debug = millis();
  }
#endif
#endif
}

GPSData NEO6M::getData() const {
  return data;
}

bool NEO6M::hasFix() const {
  return data.fix_valid;
}

// ============================================================================
// Internal Parsing
// ============================================================================

void NEO6M::process_sentence() {
#ifndef MAVLINK_ENABLED
#ifndef DISABLE_GPS_DEBUG
  // Print raw NMEA sentences for serial monitor debugging
  Serial.println(sentence);
#endif
#endif

  // Validate checksum first
  if (!validate_checksum(sentence)) {
    checksum_errors++;
    return;  // Bad checksum, discard
  }
  sentences_parsed++;

  // Make a working copy (split_fields modifies the string)
  char work[NMEA_MAX_LEN];
  strncpy(work, sentence, NMEA_MAX_LEN);
  work[NMEA_MAX_LEN - 1] = '\0';

  // Remove checksum part (*XX) for field parsing
  char* star = strchr(work, '*');
  if (star) {
    *star = '\0';
  }

  // Split into fields by comma
  field_count = split_fields(work);

  if (field_count < 1) return;

  // Route to appropriate parser based on sentence type
  if (strncmp(fields[0], "$GPGGA", 6) == 0 || strncmp(fields[0], "$GNGGA", 6) == 0) {
    parse_gga(fields, field_count);
  } else if (strncmp(fields[0], "$GPRMC", 6) == 0 || strncmp(fields[0], "$GNRMC", 6) == 0) {
    parse_rmc(fields, field_count);
  } else if (strncmp(fields[0], "$GPGSV", 6) == 0 || strncmp(fields[0], "$GNGSV", 6) == 0) {
    // Parse satellite view: extract number of satellites in view (field 3)
    if (field_count >= 4) {
      data.satellites = (uint8_t)atoi(fields[3]);
    }
  }
}

bool NEO6M::validate_checksum(const char* sent) {
  // NMEA checksum: XOR all bytes between '$' and '*' (exclusive)
  // Checksum is 2-digit hex after '*'

  // Find '$' and '*'
  const char* start = strchr(sent, '$');
  const char* star = strchr(sent, '*');

  if (!start || !star || star <= start + 1) {
    return false;
  }

  // Calculate XOR checksum
  uint8_t calc_checksum = 0;
  for (const char* p = start + 1; p < star; p++) {
    calc_checksum ^= *p;
  }

  // Parse expected checksum (2 hex digits after '*')
  if (strlen(star) < 3) {
    return false;
  }

  uint8_t expected = 0;
  for (int i = 1; i <= 2; i++) {
    char c = star[i];
    expected <<= 4;
    if (c >= '0' && c <= '9') expected |= (c - '0');
    else if (c >= 'A' && c <= 'F') expected |= (c - 'A' + 10);
    else if (c >= 'a' && c <= 'f') expected |= (c - 'a' + 10);
    else return false;
  }

  return calc_checksum == expected;
}

uint8_t NEO6M::split_fields(char* sent) {
  // Split comma-separated NMEA sentence into fields
  uint8_t count = 0;

  fields[count++] = sent;

  for (char* p = sent; *p && count < NMEA_MAX_FIELDS; p++) {
    if (*p == ',') {
      *p = '\0';
      fields[count++] = p + 1;
    }
  }

  return count;
}

// ============================================================================
// $GPGGA Parser — Position, Fix, Satellites, Altitude
// ============================================================================
//
// Format: $GPGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoid,M,age,ref*CS
// Fields:  0      1    2   3   4   5   6   7    8    9  10  11  12 13  14
//
void NEO6M::parse_gga(char** f, uint8_t count) {
  if (count < 10) return;

  // Fix quality (field 6): 0=no fix, 1=GPS, 2=DGPS
  data.fix_quality = atoi(f[6]);

  // Number of satellites (field 7)
  data.satellites = atoi(f[7]);

  // Only update position if we have a fix
  if (data.fix_quality > 0 && strlen(f[2]) > 0 && strlen(f[4]) > 0) {
    // Latitude (field 2 + 3)
    data.latitude = nmea_to_decimal(f[2], f[3]);

    // Longitude (field 4 + 5)
    data.longitude = nmea_to_decimal(f[4], f[5]);

    // Altitude above sea level in meters (field 9)
    if (strlen(f[9]) > 0) {
      data.altitude_gps = atof(f[9]);
    }

    data.fix_valid = true;
    data.last_fix_time = millis();
  }
}

// ============================================================================
// $GPRMC Parser — Speed, Course, Fix Status
// ============================================================================
//
// Format: $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,mag,E/W,mode*CS
// Fields:  0      1    2      3   4   5   6   7     8      9    10  11  12
//
void NEO6M::parse_rmc(char** f, uint8_t count) {
  if (count < 9) return;

  // Status (field 2): A=Active (valid), V=Void (invalid)
  bool rmc_valid = (f[2][0] == 'A');

  if (rmc_valid) {
    // Update position from RMC as well (redundant with GGA but useful)
    if (strlen(f[3]) > 0 && strlen(f[5]) > 0) {
      data.latitude = nmea_to_decimal(f[3], f[4]);
      data.longitude = nmea_to_decimal(f[5], f[6]);
    }

    // Speed over ground in knots (field 7)
    if (strlen(f[7]) > 0) {
      data.speed_knots = atof(f[7]);
    }

    // Course over ground in degrees (field 8)
    if (strlen(f[8]) > 0) {
      data.course = atof(f[8]);
    }

    data.fix_valid = true;
    data.last_fix_time = millis();
  }
}

// ============================================================================
// NMEA Coordinate Conversion
// ============================================================================
//
// NMEA format: DDMM.MMMM (degrees + decimal minutes)
// Convert to decimal degrees: DD + (MM.MMMM / 60.0)
// Negate if hemisphere is S or W
//
float NEO6M::nmea_to_decimal(const char* nmea_val, const char* hemisphere) {
  if (!nmea_val || strlen(nmea_val) == 0) return 0.0f;

  // Parse the full value as a float
  float raw = atof(nmea_val);

  // Extract degrees (integer part of raw / 100)
  int degrees = (int)(raw / 100.0f);

  // Extract minutes (remainder)
  float minutes = raw - (degrees * 100.0f);

  // Convert to decimal degrees
  float decimal = degrees + (minutes / 60.0f);

  // Apply hemisphere sign
  if (hemisphere && (hemisphere[0] == 'S' || hemisphere[0] == 'W')) {
    decimal = -decimal;
  }

  return decimal;
}
