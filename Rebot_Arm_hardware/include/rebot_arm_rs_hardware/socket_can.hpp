#pragma once

#include <linux/can.h>
#include <sys/types.h>
#include <chrono>
#include <cstdint>
#include <string>

namespace rebot_arm_rs_hardware
{
// Injectable transport keeps all driver tests off the real CAN interface.
class CanTransport
{
public:
  virtual ~CanTransport() = default;
  virtual void open(const std::string & interface, uint8_t host_id) = 0;
  virtual void close() noexcept = 0;
  virtual void send(const can_frame & frame) = 0;
  virtual bool receive(can_frame & frame) = 0;
};

class SocketCan : public CanTransport
{
public:
  ~SocketCan() override;
  void open(const std::string & interface, uint8_t host_id) override;
  void close() noexcept override;
  void send(const can_frame & frame) override;
  bool receive(can_frame & frame) override;
protected:
  // One nonblocking syscall, separated to exercise TX congestion without CAN hardware.
  virtual ssize_t transmit(const can_frame & frame);
private:
  int fd_{-1};
  std::chrono::steady_clock::time_point last_sent_{};
};
}  // namespace rebot_arm_rs_hardware
