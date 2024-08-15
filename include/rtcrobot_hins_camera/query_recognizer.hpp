#ifndef _QUERY_RECOGNIZER_HPP_
#define _QUERY_RECOGNIZER_HPP_

#include "crc16_check.hpp"
#include "hinson_type.hpp"
#include <iostream>
#include <vector>

namespace hinson_protocol {

class QueryRecognizer {
public:
  enum class MethodREQUEST : uint8_t {
    STOP      = 0x00,
    QUERY_ONCE= 0x01,
    QUERY_AUTO= 0x02
  };
  enum class MethodRESPONSE : uint8_t {
    FAILED = 0x00,
    SINGLE = 0x01,
    CODE22 = 0x02,
    CODE44 = 0x03,
    CODEBAR= 0x04
  };

public:
  struct Request {
    uint16_t protocol_version{uint16_t(Version::NEW)};
    uint8_t  command{uint8_t(Command::QUERY_RECOGNIZER)};
    uint8_t  method{uint8_t(MethodREQUEST::QUERY_AUTO)};

    std::vector<uint8_t> frame;

    Request() {}

    Request(MethodREQUEST mth) : method(uint8_t(mth)) {}

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
    uint16_t protocol_version{uint16_t(Version::NEW)};
    uint8_t  command{uint8_t(Command::QUERY_RECOGNIZER)};
    uint8_t  method{0x00};
    size_t   size{36};

    uint64_t id{0};
    double   x{0.0};
    double   y{0.0};
    double   theta{0.0};

    std::vector<uint8_t> frame;
    bool                 isValid{false};

    Response(std::vector<uint8_t> &response) : frame(response) {
      if (response.size() != size) {
        isValid= false;
        std::cerr << "Invalid response size" << std::endl;
        return;
      }

      if ((uint16_t(response[6]) << 8 | response[7]) != protocol_version) {
        isValid= false;
        std::cerr << "Invalid protocol version" << std::endl;
        return;
      }

      if (response[8] != command) {
        isValid= false;
        std::cerr << "Invalid command" << std::endl;
        return;
      }

      if (response[9] == uint8_t(MethodRESPONSE::FAILED)) {
        isValid= false;
        return;
      }

      id= frame[10] << 56 | frame[11] << 48 | frame[12] << 40 |
          frame[13] << 32 | frame[14] << 24 | frame[15] << 16 | frame[16] << 8 |
          frame[17];

      int32_t x= frame[18] << 24 | frame[19] << 16 | frame[20] << 8 | frame[21];
      int32_t y= frame[22] << 24 | frame[23] << 16 | frame[24] << 8 | frame[25];
      int16_t theta= frame[26] << 8 | frame[27];

      this->x    = double(x) * 0.0001;
      this->y    = double(y) * 0.0001;
      this->theta= double(theta) * 0.1;

      isValid= true;
    }
  };
};

} // namespace hinson_protocol

#endif // _QUERY_RECOGNIZER_HPP_