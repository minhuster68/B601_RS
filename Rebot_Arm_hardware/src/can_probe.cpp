#include "rebot_arm_rs_hardware/robstride_protocol.hpp"
#include "rebot_arm_rs_hardware/socket_can.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

using namespace rebot_arm_rs_hardware;  // NOLINT

namespace
{
uint8_t parse_id(const std::string & text)
{
  std::size_t used{};
  const int value = std::stoi(text, &used);
  if (used != text.size() || value < 1 || value > 254) {
    throw std::invalid_argument("IDs must be decimal integers in [1,254]");
  }
  return static_cast<uint8_t>(value);
}
}

int main(int argc, char ** argv)
{
  try {
    std::string interface = "can0";
    uint8_t host = 253;
    std::set<uint8_t> ids;
    bool scan = false;
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--help") {
        std::cout << "can_probe --interface can0 --host 253 --ids 1,2,3\n"
                  << "can_probe --interface can0 --host 253 --scan\n"
                  << "Reads 0x7019/0x701B only; never enables, disables or zeros motors.\n";
        return 0;
      }
      if (argument == "--scan") {scan = true; continue;}
      if (i + 1 >= argc) {throw std::invalid_argument("Missing argument value");}
      const std::string value = argv[++i];
      if (argument == "--interface") {interface = value;}
      else if (argument == "--host") {host = parse_id(value);}
      else if (argument == "--ids") {
        std::istringstream stream(value);
        std::string text;
        while (std::getline(stream, text, ',')) {ids.insert(parse_id(text));}
      } else {throw std::invalid_argument("Unknown argument: " + argument);}
    }
    if (scan && !ids.empty()) {throw std::invalid_argument("Choose --scan OR --ids");}
    if (scan) {
      for (unsigned id = 1; id <= 254; ++id) {
        if (id != host) {ids.insert(static_cast<uint8_t>(id));}
      }
    }
    if (ids.empty() || ids.count(host)) {
      throw std::invalid_argument("Supply --ids or --scan; motor IDs must differ from --host");
    }
    SocketCan bus;
    bus.open(interface, host);
    std::map<std::pair<uint8_t, uint16_t>, float> values;
    const auto start = std::chrono::steady_clock::now();
    auto last_query = start - std::chrono::seconds(1);
    auto next_id = ids.begin();
    const auto duration = std::chrono::milliseconds(scan ? ids.size() * 10 + 500 : 2000);
    while (std::chrono::steady_clock::now() - start < duration) {
      const auto current = std::chrono::steady_clock::now();
      if (scan && next_id != ids.end() && current - last_query >= std::chrono::milliseconds(10)) {
        bus.send(protocol::read_parameter(host, *next_id, protocol::kPosition));
        bus.send(protocol::read_parameter(host, *next_id, protocol::kVelocity));
        ++next_id;
        last_query = current;
      } else if (!scan && current - last_query >= std::chrono::milliseconds(100)) {
        for (const auto id : ids) {
          for (const auto index : {protocol::kPosition, protocol::kVelocity}) {
            if (!values.count({id, index})) {bus.send(protocol::read_parameter(host, id, index));}
          }
        }
        last_query = current;
      }
      can_frame frame{};
      for (std::size_t count = 0; count < 256 && bus.receive(frame); ++count) {
        const auto reply = protocol::decode(frame, host);
        if (reply && reply->type == 17 && ids.count(reply->motor_id)) {
          if (reply->flags) {throw std::runtime_error("Motor rejected a parameter read");}
          if (reply->index == protocol::kPosition || reply->index == protocol::kVelocity) {
            if (!std::isfinite(reply->as_float())) {throw std::runtime_error("Non-finite motor response");}
            const auto key = std::make_pair(reply->motor_id, reply->index);
            if (!values.count(key)) {
              values[key] = reply->as_float();
              std::cout << "motor_id=" << unsigned(reply->motor_id)
                        << (reply->index == protocol::kPosition ? " mechPos_rad=" : " mechVel_rad_s=")
                        << reply->as_float() << std::endl;
            }
          }
        }
      }
      if (values.size() == ids.size() * 2) {return 0;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (scan && !values.empty()) {
      std::set<uint8_t> found;
      for (const auto & value : values) {found.insert(value.first.first);}
      std::cout << "Responding motor IDs:";
      for (const auto id : found) {std::cout << ' ' << unsigned(id);}
      std::cout << "\nIDs do not identify physical joints; match each ID to its installed motor.\n";
      return 0;
    }
    std::cerr << "Missing replies: check selected IDs, private protocol, host ID and bus settings\n";
    return 2;
  } catch (const std::exception & error) {
    std::cerr << "can_probe: " << error.what() << '\n';
    return 1;
  }
}
