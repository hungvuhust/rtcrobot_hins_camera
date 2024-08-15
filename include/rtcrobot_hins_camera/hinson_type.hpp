#ifndef _HINSON_TYPE_HPP_
#define _HINSON_TYPE_HPP_

#include <cstdint>

namespace hinson_protocol {

static void print_debug_recv(uint8_t *buffer, ssize_t n) {
  printf("Server response[%ld]: ", n);
  for (size_t i= 0; i < n; i++) {
    printf("%02X ", buffer[i] & 0xFF);
  }
  printf("\n");
}

enum class Version : uint16_t { OLD= 0x0000, NEW= 0x0200 };

enum class Command : uint8_t {
  QUERY_RECOGNIZER= 0x00,
  READ_PARAMETER  = 0x01,
  CHANGE_PARAMETER= 0x02,
  QUERY_IMAGE     = 0x03,
  SET_CODE_LENGTH = 0x06,
  SET_TYPE_CODE   = 0x0B,
};

} // namespace hinson_protocol

#endif // _HINSON_TYPE_HPP_