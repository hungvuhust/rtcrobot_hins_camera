#ifndef _SET_CODE_type_CPP_
#define _SET_CODE_type_CPP_

#include "crc16_check.hpp"
#include "hinson_type.hpp"
#include <cstdint>
#include <iostream>
#include <vector>

namespace hinson_protocol {

class SetTypeCode {

  enum class TYPE : uint16_t {
    DM10X10= 0x0000,
    DM12X12= 0x0100,

  };

public:
  struct Request {
    uint16_t protocol_version{uint16_t(Version::OLD)};
    uint8_t  command{uint8_t(Command::SET_TYPE_CODE)};
    uint16_t type{uint16_t(TYPE::DM12X12)};

    std::vector<uint8_t> frame;

    Request(uint16_t l) : type(l) {}

    std::vector<uint8_t> getFrame() {
      // Add the header to the frame
      std::vector<uint8_t> header;
      frame.clear();

      header.insert(header.end(), HEADER_FRAME, HEADER_FRAME + 6);
      frame.insert(frame.end(), header.begin(), header.end());
      // Add protocol version to the frame
      frame.push_back((protocol_version >> 8) & 0xFF);
      frame.push_back(protocol_version & 0xFF);
      // Add command to the frame
      frame.push_back(command);

      // Add type to the frame
      frame.push_back(type & 0xFF);
      frame.push_back((type >> 8) & 0xFF);

      //   CRC16 check
      uint16_t crc= CRC16(frame.data(), frame.size());
      frame.push_back(crc & 0xFF);
      frame.push_back((crc >> 8) & 0xFF);
      return frame;
    }
  };

  struct Response {
    uint16_t protocol_version{uint16_t(Version::OLD)};
    uint8_t  command{uint8_t(Command::SET_TYPE_CODE)};

    bool isValid;
    Response(std::vector<uint8_t> &response, uint16_t type) {
      if ((uint16_t(response[6]) << 8 | response[7]) != protocol_version) {
        isValid= false;
        std::cerr << "Invalid protocol version" << std::endl;
        return;
      }

      if (response[8] != command) {
        isValid= false;
        std::cerr << "Invalid method" << std::endl;
        return;
      }

      if (uint16_t(response[10] << 8 | response[9]) != 0x00) {
        std::cerr << "ChangeParameter response: Failed" << std::endl;
        isValid= false;
        return;
      }

      isValid= true;
    }
  };
};
} // namespace hinson_protocol

#endif //_SET_CODE_type_CPP_