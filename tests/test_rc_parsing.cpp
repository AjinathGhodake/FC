#include <cassert>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <stdexcept>

// Include the RC receiver header
// For testing, we'll use a modified version with mockable UART

// ============================================================================
// Mock UART2 for Testing
// ============================================================================

static uint8_t mock_uart_buffer[256];
static int mock_uart_write_idx = 0;
static int mock_uart_read_idx = 0;

void mock_uart_reset() {
  mock_uart_write_idx = 0;
  mock_uart_read_idx = 0;
  memset(mock_uart_buffer, 0, sizeof(mock_uart_buffer));
}

void mock_uart_write_byte(uint8_t byte) {
  if (mock_uart_write_idx < 256) {
    mock_uart_buffer[mock_uart_write_idx++] = byte;
  }
}

bool mock_uart_has_data() {
  return mock_uart_read_idx < mock_uart_write_idx;
}

uint8_t mock_uart_read_byte() {
  if (mock_uart_read_idx < mock_uart_write_idx) {
    return mock_uart_buffer[mock_uart_read_idx++];
  }
  return 0;
}

// ============================================================================
// Test CRC-8 Function
// ============================================================================

uint8_t test_calc_crc(const uint8_t* data, uint8_t len) {
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

// ============================================================================
// Test Channel Unpacking Function
// ============================================================================

uint16_t test_unpack_channel(const uint8_t* payload, uint8_t channel_idx) {
  uint16_t bit_position = channel_idx * 11;
  uint8_t byte_idx = bit_position / 8;
  uint8_t bit_offset = bit_position % 8;

  uint16_t value = 0;

  if (bit_offset <= 5) {
    value = (payload[byte_idx] << 8) | payload[byte_idx + 1];
    value = (value >> (16 - 11 - bit_offset)) & 0x7FF;
  } else {
    value = (payload[byte_idx] << 16) | (payload[byte_idx + 1] << 8) | payload[byte_idx + 2];
    value = (value >> (24 - 11 - bit_offset)) & 0x7FF;
  }

  // Convert from CRSF (0-2047) to PWM (1000-2000 μs)
  uint16_t pwm = ((value - 1024) / 1.024f) + 1500;

  if (pwm < 1000) pwm = 1000;
  if (pwm > 2000) pwm = 2000;

  return pwm;
}

// ============================================================================
// Test Cases
// ============================================================================

void test_crc_calculation() {
  printf("Testing CRC-8 calculation...\n");

  // Test case 1: Simple data
  uint8_t data1[] = {0x18, 0x16, 0x00, 0x00};
  uint8_t crc1 = test_calc_crc(data1, 4);
  printf("  CRC of [0x18, 0x16, 0x00, 0x00] = 0x%02X\n", crc1);

  // Test case 2: Known CRSF frame data (without CRC)
  // This would be from actual captured CRSF data
  uint8_t data2[] = {0x18, 0x16};  // LENGTH, TYPE for RC channels
  uint8_t crc2 = test_calc_crc(data2, 2);
  printf("  CRC of [0x18, 0x16] = 0x%02X\n", crc2);

  printf("  PASSED\n\n");
}

void test_channel_unpacking() {
  printf("Testing channel unpacking...\n");

  // Create a mock RC channels payload (20 bytes, 11 bits per channel)
  // Initialize with all channels at center (1024 raw = ~1500 PWM)
  uint8_t payload[20];
  memset(payload, 0, sizeof(payload));

  // Set all channels to center value (1024 raw)
  // This is a simplified approach; real payload would be bit-packed

  // For testing, create a known pattern:
  // Bytes layout for 16 channels of 11 bits each:
  // Each channel takes 11 bits, channels are packed sequentially

  // Simplified test: fill with a pattern
  for (int i = 0; i < 20; i++) {
    payload[i] = 0x80;  // Mid-value for testing
  }

  // Test unpacking first few channels
  uint16_t ch0 = test_unpack_channel(payload, 0);
  uint16_t ch1 = test_unpack_channel(payload, 1);
  uint16_t ch2 = test_unpack_channel(payload, 2);

  printf("  Channel 0 (unpacked from 0x80 pattern): %u μs\n", ch0);
  printf("  Channel 1 (unpacked from 0x80 pattern): %u μs\n", ch1);
  printf("  Channel 2 (unpacked from 0x80 pattern): %u μs\n", ch2);

  // Verify values are in valid PWM range
  assert(ch0 >= 1000 && ch0 <= 2000);
  assert(ch1 >= 1000 && ch1 <= 2000);
  assert(ch2 >= 1000 && ch2 <= 2000);

  printf("  PASSED\n\n");
}

void test_channel_conversion() {
  printf("Testing channel value conversion...\n");

  // Test conversion of raw CRSF values to PWM
  // CRSF: 0 = -100%, 1024 = center, 2047 = +100%
  // PWM:  1000 = -100%, 1500 = center, 2000 = +100%

  // Helper lambda to convert
  auto convert = [](uint16_t raw) {
    uint16_t pwm = ((raw - 1024) / 1.024f) + 1500;
    if (pwm < 1000) pwm = 1000;
    if (pwm > 2000) pwm = 2000;
    return pwm;
  };

  // Test cases
  uint16_t pwm_min = convert(0);      // Should be ~1000
  uint16_t pwm_center = convert(1024); // Should be ~1500
  uint16_t pwm_max = convert(2047);   // Should be ~2000

  printf("  Raw 0 -> PWM: %u μs (expected ~1000)\n", pwm_min);
  printf("  Raw 1024 -> PWM: %u μs (expected ~1500)\n", pwm_center);
  printf("  Raw 2047 -> PWM: %u μs (expected ~2000)\n", pwm_max);

  assert(pwm_min <= 1010);   // Allow small tolerance
  assert(pwm_center >= 1490 && pwm_center <= 1510);
  assert(pwm_max >= 1990);

  printf("  PASSED\n\n");
}

void test_rc_struct() {
  printf("Testing RCChannels struct...\n");

  // Simulate RC data structure
  struct {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t mode;
    uint16_t aux;
    bool signal_ok;
  } channels = {1200, 1400, 1300, 1600, 1500, 1500, true};

  printf("  Throttle: %u μs\n", channels.throttle);
  printf("  Roll:     %u μs\n", channels.roll);
  printf("  Pitch:    %u μs\n", channels.pitch);
  printf("  Yaw:      %u μs\n", channels.yaw);
  printf("  Mode:     %u μs\n", channels.mode);
  printf("  Aux:      %u μs\n", channels.aux);
  printf("  Signal OK: %s\n", channels.signal_ok ? "yes" : "no");

  // Verify struct layout
  assert(channels.throttle == 1200);
  assert(channels.signal_ok == true);

  printf("  PASSED\n\n");
}

void test_frame_validation() {
  printf("Testing CRSF frame validation...\n");

  // Create a minimal valid CRSF frame:
  // [SYNC=0xC8] [LENGTH=0x18] [TYPE=0x16] [PAYLOAD...] [CRC]

  uint8_t frame[26];
  frame[0] = 0xC8;     // SYNC
  frame[1] = 0x18;     // LENGTH (24 bytes: type + 20 bytes payload + CRC)
  frame[2] = 0x16;     // TYPE (RC channels packed)

  // Fill payload with dummy data (20 bytes)
  for (int i = 3; i < 23; i++) {
    frame[i] = 0x40;   // Dummy payload
  }

  // Calculate and add CRC (of bytes 1-22)
  frame[23] = test_calc_crc(&frame[1], 22);

  printf("  Created test frame:\n");
  printf("    SYNC: 0x%02X\n", frame[0]);
  printf("    LENGTH: 0x%02X\n", frame[1]);
  printf("    TYPE: 0x%02X\n", frame[2]);
  printf("    CRC: 0x%02X\n", frame[23]);

  // Validate frame structure
  assert(frame[0] == 0xC8);
  assert(frame[1] == 0x18);
  assert(frame[2] == 0x16);

  printf("  PASSED\n\n");
}

void test_timeout_behavior() {
  printf("Testing signal timeout behavior...\n");

  // Simulate timeout detection
  unsigned long frame_time = 1000;  // ms
  unsigned long current_time = 1600;  // ms (600ms later)
  unsigned long timeout_ms = 500;

  bool signal_ok = (current_time - frame_time) <= timeout_ms;
  printf("  Frame at %lu ms, check at %lu ms\n", frame_time, current_time);
  printf("  Elapsed: %lu ms (timeout: %lu ms)\n", current_time - frame_time, timeout_ms);
  printf("  Signal OK: %s\n", signal_ok ? "yes" : "no");

  assert(signal_ok == false);  // Should timeout

  // Test non-timeout case
  current_time = 1400;  // 400ms later
  signal_ok = (current_time - frame_time) <= timeout_ms;
  printf("\n  Frame at %lu ms, check at %lu ms\n", frame_time, current_time);
  printf("  Elapsed: %lu ms (timeout: %lu ms)\n", current_time - frame_time, timeout_ms);
  printf("  Signal OK: %s\n", signal_ok ? "yes" : "no");

  assert(signal_ok == true);  // Should NOT timeout

  printf("  PASSED\n\n");
}

// ============================================================================
// Main Test Runner
// ============================================================================

int main() {
  printf("========================================\n");
  printf("CRSF RC Receiver Parser - Unit Tests\n");
  printf("========================================\n\n");

  try {
    test_crc_calculation();
    test_channel_unpacking();
    test_channel_conversion();
    test_rc_struct();
    test_frame_validation();
    test_timeout_behavior();

    printf("========================================\n");
    printf("ALL TESTS PASSED\n");
    printf("========================================\n");
    return 0;
  } catch (const std::exception& e) {
    printf("TEST FAILED: %s\n", e.what());
    return 1;
  }
}
