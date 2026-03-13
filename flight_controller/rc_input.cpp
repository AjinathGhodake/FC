#include <Arduino.h>
#include "rc_input.h"

// Use HardwareSerial for UART2 (PA3=RX, PA2=TX)
HardwareSerial Serial2(PA3, PA2);  // RX, TX pins

// ============================================================================
// Constructor
// ============================================================================

CRSFReceiver::CRSFReceiver()
  : last_frame_time(0), buffer_index(0), frame_in_progress(false) {
  // Initialize channels to mid-range (1500 us) with no signal
  channels.throttle = 1500;
  channels.roll = 1500;
  channels.pitch = 1500;
  channels.yaw = 1500;
  channels.mode = 1500;
  channels.aux = 1500;
  channels.signal_ok = false;
}

// ============================================================================
// Public Methods
// ============================================================================

void CRSFReceiver::begin() {
  // Initialize UART2 at 420k baud via Arduino HardwareSerial API
  Serial2.begin(CRSF_BAUDRATE);
  last_frame_time = millis();
}

bool CRSFReceiver::update() {
  unsigned long current_time = millis();
  uint8_t byte;

  // Check signal timeout
  if ((current_time - last_frame_time) > CRSF_SIGNAL_TIMEOUT_MS) {
    channels.signal_ok = false;
  }

  // Read available bytes from UART2
  while (read_byte(byte)) {
    if (!frame_in_progress) {
      // Looking for sync byte
      if (byte == CRSF_SYNC_BYTE) {
        frame_in_progress = true;
        buffer_index = 0;
        frame_buffer[buffer_index++] = byte;
      }
    } else {
      frame_buffer[buffer_index++] = byte;

      // Frame format:
      // [SYNC] [LENGTH] [TYPE] [PAYLOAD...] [CRC]
      // LENGTH = number of bytes after LENGTH (not including SYNC)
      // Minimum frame: [SYNC] [LEN] [TYPE] [CRC] = 4 bytes

      if (buffer_index >= 4) {
        uint8_t frame_len = frame_buffer[1];  // Length field (after SYNC)

        // Safety check
        if (frame_len < 2 || frame_len > CRSF_MAX_PACKET_LEN - 2) {
          frame_in_progress = false;
          buffer_index = 0;
          continue;
        }

        // Check if complete frame received (SYNC + LEN + frame_len bytes)
        if (buffer_index >= (2 + frame_len)) {
          // Validate and process frame
          if (process_frame(frame_buffer, buffer_index)) {
            last_frame_time = current_time;
            channels.signal_ok = true;
          }

          frame_in_progress = false;
          buffer_index = 0;
          return true;
        }
      }

      // Prevent buffer overflow
      if (buffer_index >= CRSF_MAX_PACKET_LEN) {
        frame_in_progress = false;
        buffer_index = 0;
      }
    }
  }

  return false;
}

RCChannels CRSFReceiver::getChannels() const {
  return channels;
}

bool CRSFReceiver::isConnected() const {
  return channels.signal_ok;
}

// ============================================================================
// Private Methods
// ============================================================================

bool CRSFReceiver::read_byte(uint8_t& byte) {
  if (Serial2.available()) {
    byte = Serial2.read();
    return true;
  }
  return false;
}

bool CRSFReceiver::process_frame(const uint8_t* frame, uint8_t len) {
  // Frame structure:
  // [0] = SYNC (0xC8)
  // [1] = LENGTH (bytes after this, including type and CRC)
  // [2] = TYPE
  // [3..n-1] = PAYLOAD
  // [n] = CRC

  if (len < 4) {
    return false;
  }

  if (frame[0] != CRSF_SYNC_BYTE) {
    return false;
  }

  uint8_t frame_length = frame[1];
  uint8_t frame_type = frame[2];

  // CRC is the last byte
  uint8_t crc_received = frame[len - 1];

  // Calculate CRC of [LENGTH TYPE PAYLOAD] (excluding SYNC and CRC)
  uint8_t crc_calc = calc_crc(&frame[1], len - 3);

  if (crc_calc != crc_received) {
    return false;  // CRC mismatch
  }

  // Process RC channels frame
  if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
    // RC channels frame payload format (20 bytes):
    // 11 bits per channel, 16 channels packed
    // We extract the first 6 channels (throttle, roll, pitch, yaw, mode, aux)

    if (len < 2 + CRSF_RC_CHANNELS_LEN) {
      return false;  // Frame too short
    }

    const uint8_t* payload = &frame[3];  // Skip SYNC, LENGTH, TYPE

    // Unpack 6 channels (11 bits each = 2 bytes per channel rounded up)
    channels.throttle = unpack_channel(payload, 0);
    channels.roll = unpack_channel(payload, 1);
    channels.pitch = unpack_channel(payload, 2);
    channels.yaw = unpack_channel(payload, 3);
    channels.mode = unpack_channel(payload, 4);
    channels.aux = unpack_channel(payload, 5);

    return true;
  }

  // Ignore other frame types (link stats, etc.)
  return false;
}

uint8_t CRSFReceiver::calc_crc(const uint8_t* data, uint8_t len) {
  // CRSF uses CRC-8 with polynomial 0xD5 (CRC-8 Dallas/Maxim)
  // Initial value = 0
  uint8_t crc = 0;

  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc = crc << 1;
      }
    }
  }

  return crc;
}

uint16_t CRSFReceiver::unpack_channel(const uint8_t* payload, uint8_t channel_idx) {
  // CRSF packs 16 channels in 22 bytes (11 bits per channel)
  // Channel layout (11 bits each):
  // Ch0: bits 0-10 (bytes 0-1)
  // Ch1: bits 11-21 (bytes 1-2)
  // Ch2: bits 22-32 (bytes 2-3)
  // ... and so on

  // Bit packing formula:
  uint16_t bit_position = channel_idx * 11;
  uint8_t byte_idx = bit_position / 8;
  uint8_t bit_offset = bit_position % 8;

  uint16_t value = 0;

  if (bit_offset <= 5) {
    // Channel fits within 2 bytes
    value = (payload[byte_idx] << 8) | payload[byte_idx + 1];
    value = (value >> (16 - 11 - bit_offset)) & 0x7FF;  // Extract 11 bits
  } else {
    // Channel spans 3 bytes
    value = (payload[byte_idx] << 16) | (payload[byte_idx + 1] << 8) | payload[byte_idx + 2];
    value = (value >> (24 - 11 - bit_offset)) & 0x7FF;  // Extract 11 bits
  }

  // Convert from CRSF range (0-2047 raw) to PWM range (1000-2000 us)
  // CRSF: 0 = -100%, 1024 = center (0%), 2047 = +100%
  // PWM:  1000 us = -100%, 1500 us = center, 2000 us = +100%
  uint16_t pwm = ((value - 1024) / 1.024f) + 1500;

  // Clamp to valid range
  if (pwm < 1000) pwm = 1000;
  if (pwm > 2000) pwm = 2000;

  return pwm;
}
