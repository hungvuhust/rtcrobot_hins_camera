#ifndef _READ_PARAMETER_HPP_
#define _READ_PARAMETER_HPP_

#include "crc16_check.hpp"
#include "hinson_type.hpp"
#include <cstdint>
#include <iostream>
#include <vector>

namespace hinson_protocol {

class ReadParameter {
public:
  struct Request {
    uint16_t protocol_version{uint16_t(Version::OLD)};
    uint8_t  command{uint8_t(Command::READ_PARAMETER)};

    std::vector<uint8_t> frame;

    Request() {}

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
      //   CRC16 check
      uint16_t crc= CRC16(frame.data(), frame.size());
      frame.push_back(crc & 0xFF);
      frame.push_back((crc >> 8) & 0xFF);
      return frame;
    }
  };

  struct Response {
    uint16_t protocol_version{uint16_t(Version::OLD)};
    uint8_t  command{uint8_t(Command::READ_PARAMETER)};

    uint8_t  gain;
    uint16_t exposure;
    uint8_t  pwm;
    uint8_t  corrosion;

    size_t size{16};

    bool isValid{false};

    Response(std::vector<uint8_t> &response) {
      if (response.size() != size) {
        isValid= false;
        std::cerr << "Invalid response size" << std::endl;
        return;
      }

      if (!isValidFrame(response.data(), response.size())) {
        isValid= false;
        std::cerr << "Invalid frame" << std::endl;
        return;
      }

      if (!CRCCheck(response.data(), response.size())) {
        isValid= false;
        std::cerr << "Invalid CRC" << std::endl;
        return;
      }

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

      gain     = response[9];
      exposure = (response[11] << 8) | response[10];
      pwm      = response[12];
      corrosion= response[13];

      isValid= true;
    }
  };
};
} // namespace hinson_protocol

#endif // _READ_PARAMETER_HPP_