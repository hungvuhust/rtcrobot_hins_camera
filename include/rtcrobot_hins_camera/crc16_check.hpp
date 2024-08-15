#ifndef _CRC16_CHECK_HPP_
#define _CRC16_CHECK_HPP_

#include <cstdint>
#include <cstring>
#include <iostream>

namespace hinson_protocol {

#define CRC16_POLYNOMIAL 0xA001
#define CRC16_INITIAL    0xFFFF
#define HEADER_FRAME     "HSTDCS"

/**
 * @brief Performs CRC check on a given command.
 *
 * This function calculates the CRC (Cyclic Redundancy Check) value for a given
 * command and compares it with the CRC value appended at the end of the
 * command. If the calculated CRC matches the appended CRC, the function returns
 * true; otherwise, it returns false.
 *
 * @param command Pointer to the command buffer.
 * @param len Length of the command buffer.
 * @return True if the CRC check passes, false otherwise.
 */
static bool CRCCheck(const uint8_t *command, int len) {
  uint16_t wCrc    = CRC16_INITIAL;
  uint16_t wPolynom= CRC16_POLYNOMIAL;

  for (int i= 0; i < len - 2; i++) {
    wCrc^= command[i];
    for (int j= 0; j < 8; j++) {
      wCrc= (wCrc & 0x0001) ? (wCrc >> 1) ^ wPolynom : wCrc >> 1;
    }
  }

  uint8_t high= (wCrc >> 8) & 0xFF;
  uint8_t low = wCrc & 0xFF;

  return (low == command[len - 2] && high == command[len - 1]);
}

/**
 * @brief Checks if a frame is valid.
 *
 * This function compares the first 6 bytes of the frame with a predefined
 * message to determine if the frame is valid.
 *
 * @param frame A pointer to the frame data.
 * @param frameSize The size of the frame in bytes.
 * @return true if the frame is valid, false otherwise.
 */
static bool isValidFrame(const uint8_t *frame, size_t frameSize) {
  uint8_t message[]= {0x48, 0x53, 0x54, 0x44, 0x43, 0x53}; // HSTDCS

  // Kiểm tra nếu kích thước frame nhỏ hơn 6 thì không hợp lệ
  if (frameSize < 6) {
    return false;
  }

  // So sánh 6 byte đầu tiên của frame với message2
  return std::memcmp(frame, message, 6) == 0;
}

/**
 * Calculates the CRC16 checksum for a given command.
 *
 * @param command The command for which to calculate the CRC16 checksum.
 * @param len The length of the command.
 * @return The CRC16 checksum value.
 */
static uint16_t CRC16(const uint8_t *command, int len) {
  uint16_t wCrc    = CRC16_INITIAL;
  uint16_t wPolynom= CRC16_POLYNOMIAL;

  for (int i= 0; i < len; i++) {
    wCrc^= command[i];
    for (int j= 0; j < 8; j++) {
      wCrc= (wCrc & 0x0001) ? (wCrc >> 1) ^ wPolynom : wCrc >> 1;
    }
  }

  return wCrc;
}
} // namespace hinson_protocol

#endif // _CRC16_CHECK_HPP_