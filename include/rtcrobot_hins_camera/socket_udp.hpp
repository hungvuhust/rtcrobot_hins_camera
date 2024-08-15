#ifndef _SOCKET_UDP_HPP_
#define _SOCKET_UDP_HPP_

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

class SocketUDP {
public:
  SocketUDP(const std::string &address, int port) {
    sockfd= socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
      std::cerr << "Error creating socket" << std::endl;
      exit(1);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family= AF_INET;
    serverAddr.sin_port  = htons(port);
    inet_pton(AF_INET, address.c_str(), &serverAddr.sin_addr);
  }

  ~SocketUDP() { close(sockfd); }

  int getFd() { return sockfd; }

  bool send(const uint8_t *message, size_t size) {
    return sendto(
               sockfd, message, size, 0, (const struct sockaddr *)&serverAddr,
               sizeof(serverAddr)) != -1;
  }

  bool send(const std::vector<uint8_t> &message) {
    return send(message.data(), message.size());
  }

  ssize_t read(uint8_t *buffer, size_t size) {
    return recv(sockfd, buffer, size, 0);
  }

  int                sockfd;
  struct sockaddr_in serverAddr;
};

#endif