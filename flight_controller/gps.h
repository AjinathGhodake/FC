#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

// ============================================================================
// GPS Data Structure
// ============================================================================

struct GPSData {
  float latitude;           // Decimal degrees (positive = N, negative = S)
  float longitude;          // Decimal degrees (positive = E, negative = W)
  float altitude_gps;       // Meters above sea level (from GGA)
  float speed_knots;        // Ground speed in knots (from RMC)
  float course;             // Track angle in degrees (from RMC)
  uint8_t satellites;       // Number of satellites in use
  uint8_t fix_quality;      // 0=no fix, 1=GPS fix, 2=DGPS fix
  bool fix_valid;           // true if GPS has a valid fix
  unsigned long last_fix_time;  // millis() timestamp of last valid fix
};

// ============================================================================
// NMEA Parser Constants
// ============================================================================

#define GPS_BAUDRATE      9600
#define NMEA_MAX_LEN      83    // Max NMEA sentence length (82 chars + null)
#define NMEA_MAX_FIELDS   20    // Max fields in a sentence
#define GPS_FIX_TIMEOUT   3000  // 3 seconds without fix = no fix

// ============================================================================
// NEO-6M GPS Receiver Class
// ============================================================================

class NEO6M {
public:
  NEO6M();

  // Initialize UART1 (PA9=TX, PA10=RX) at 9600 baud
  void begin();

  // Read available bytes and parse NMEA sentences (call in main loop)
  void update();

  // Get current GPS data
  GPSData getData() const;

  // Check if GPS has a valid fix
  bool hasFix() const;

private:
  GPSData data;

  // NMEA sentence buffer
  char sentence[NMEA_MAX_LEN];
  uint8_t sentence_index;
  bool sentence_started;

  // Field parsing
  char* fields[NMEA_MAX_FIELDS];
  uint8_t field_count;

  // Debug counters
  uint32_t bytes_received;
  uint16_t sentences_parsed;
  uint16_t checksum_errors;

  // Internal parsing methods
  void process_sentence();
  bool validate_checksum(const char* sentence);
  uint8_t split_fields(char* sentence);

  // Sentence-specific parsers
  void parse_gga(char** fields, uint8_t count);
  void parse_rmc(char** fields, uint8_t count);

  // Helper: Convert NMEA lat/lon (DDMM.MMMM) to decimal degrees
  float nmea_to_decimal(const char* nmea_val, const char* hemisphere);
};

#endif // GPS_H
