#ifndef _CHANGE_PARAMETER_HPP_
#define _CHANGE_PARAMETER_HPP_

#include "crc16_check.hpp"
#include "hinson_type.hpp"
#include <cstdint>
#include <iostream>
#include <vector>

namespace hinson_protocol {

class ChangeParameter {
public:
  struct Request {
    uint16_t protocol_version{uint16_t(Version::OLD)};
    uint8_t  command{uint8_t(Command::CHANGE_PARAMETER)};
    uint8_t  gain{246};
    uint16_t exposure{33};
    uint8_t  pwm{80};
    uint8_t  corrosion{1};

    std::vector<uint8_t> frame;

    Request(uint8_t gain, uint16_t exposure, uint8_t pwm, uint8_t corrosion)
        : gain(gain), exposure(exposure), pwm(pwm), corrosion(corrosion) {}

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
      // Add gain to the frame
      frame.push_back(gain);
      // Add exposure to the frame
      frame.push_back(exposure & 0xFF);
      frame.push_back((exposure >> 8) & 0xFF);
      // Add pwm to the frame
      frame.push_back(pwm);
      // Add corrosion to the frame
      frame.push_back(corrosion);
      //   CRC16 check
      uint16_t crc= CRC16(frame.data(), frame.size());
      frame.push_back(crc & 0xFF);
      frame.push_back((crc >> 8) & 0xFF);
      return frame;
    }
  };

  struct Response {
    uint16_t protocol_version{uint16_t(Version::OLD)};
    uint8_t  command{uint8_t(Command::CHANGE_PARAMETER)};

    bool isValid;
    Response(std::vector<uint8_t> &response) {
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

      if (response[9] == 0x00) {
        std::cerr << "ChangeParameter response: Failed" << std::endl;
        isValid= false;
        return;
      }

      isValid= true;
    }
  };
};
} // namespace hinson_protocol

#endif // _CHANGE_PARAMETER_HPP_