#ifndef RTT_CHANNEL_H
#define RTT_CHANNEL_H

#include <string>

namespace proto {

struct Channel {
  std::string name;
  std::string ip;
  std::string port;

  Channel() =default;
  Channel(std::string name, std::string ip, std::string port);
  Channel(const Channel & other);
  Channel& operator=(Channel const&) = default;

  std::string getAddress(const std::string & _ip, const std::string & _port);
  std::string getSubscribeAddress();
  std::string getPublishAddress();

  bool operator == (const Channel & other);
  bool operator != (const Channel & other);

  std::string toInfoString(bool isPublisher);
};

} // proto

#endif //RTT_CHANNEL_H
