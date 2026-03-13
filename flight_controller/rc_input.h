#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <Arduino.h>

// ============================================================================
// RC Channel Data Structure
// ============================================================================

struct RCChannels {
  uint16_t throttle;   // Channel 0: Throttle (1000-2000 us)
  uint16_t roll;       // Channel 1: Roll (1000-2000 us)
  uint16_t pitch;      // Channel 2: Pitch (1000-2000 us)
  uint16_t yaw;        // Channel 3: Yaw (1000-2000 us)
  uint16_t mode;       // Channel 4: Mode switch (1000-2000 us)
  uint16_t aux;        // Channel 5: Auxiliary (1000-2000 us)
  bool signal_ok;      // Signal presence flag
};

// ============================================================================
// CRSF Protocol Constants
// ============================================================================

#define CRSF_BAUDRATE 420000              // ELRS CRSF baud rate
#define CRSF_FRAME_PERIOD_US 4000         // ~4ms per frame
#define CRSF_SIGNAL_TIMEOUT_MS 500        // 500ms timeout for signal loss
#define CRSF_MAX_PACKET_LEN 64            // Max CRSF frame length

// CRSF Frame types
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16    // RC channels (11 bits per channel)
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14       // Link statistics

// CRSF Address and frame structure
#define CRSF_SYNC_BYTE 0xC8               // CRSF frame sync byte
#define CRSF_FRAME_HEADER_LEN 4           // Sync + Len + Type + CRC
#define CRSF_RC_CHANNELS_LEN 22           // RC channels frame payload (2 + 20)

// ============================================================================
// CRSF Receiver Class
// ============================================================================

class CRSFReceiver {
public:
  CRSFReceiver();

  // Initialize UART2 (PA2=TX, PA3=RX) at 420k baud
  void begin();

  // Update receiver state - call in main loop
  // Returns true if new valid frame received
  bool update();

  // Get current RC channel values
  RCChannels getChannels() const;

  // Check if signal is currently valid
  bool isConnected() const;

private:
  // Internal state
  RCChannels channels;
  unsigned long last_frame_time;
  uint8_t frame_buffer[CRSF_MAX_PACKET_LEN];
  uint8_t buffer_index;
  bool frame_in_progress;

  // Helper functions
  bool read_byte(uint8_t& byte);
  bool process_frame(const uint8_t* frame, uint8_t len);
  uint8_t calc_crc(const uint8_t* data, uint8_t len);
  uint16_t unpack_channel(const uint8_t* payload, uint8_t channel_idx);
};

#endif // RC_INPUT_H
