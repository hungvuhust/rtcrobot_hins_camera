#ifndef _QUERY_IMAGE_HPP_
#define _QUERY_IMAGE_HPP_

#include "crc16_check.hpp"
#include "hinson_type.hpp"
#include <b64/decode.h> // Thư viện libb64
#include <iostream>
#include <nlohmann/json.hpp>
#include <vector>

namespace hinson_protocol {

using json= nlohmann::json;

class QueryImage {
public:
  struct Request {
    uint16_t protocol_version{uint16_t(Version::OLD)};
    uint8_t  command{uint8_t(Command::QUERY_IMAGE)};
    uint8_t  method{0x00};

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
      // Add method to the frame
      frame.push_back(method);
      //   CRC16 check
      uint16_t crc= CRC16(frame.data(), frame.size());
      frame.push_back(crc & 0xFF);
      frame.push_back((crc >> 8) & 0xFF);
      return frame;
    }
  };

  struct Response {
    uint16_t protocol_version{0x0000};
    uint8_t  command{uint8_t(Command::QUERY_IMAGE)};
    uint8_t  method{0x00};

    std::vector<uint8_t> frame;
    std::string          map;
    bool                 isValid{false};

    Response(std::vector<uint8_t> &response) : frame(response) {
      //   if (response[0] != 0x74 or re)
      json j= json::parse(frame);
      // check if j have "map"
      if (j.find("map") != j.end()) {
        map    = j["map"];
        isValid= true;
      } else {
        isValid= false;
      }
    }
    Response(uint8_t *buffer, ssize_t n) {
      frame.clear();
      frame.insert(frame.end(), buffer, buffer + n);
      json j= json::parse(buffer);
      // check if j have "map"
      if (j.find("map") != j.end()) {
        map    = j["map"];
        isValid= true;
      } else {
        isValid= false;
      }
    }
  };
};

} // namespace hinson_protocol

#endif // _QUERY_IMAGE_HPP_