#include "rebot_arm_rs_hardware/socket_can.hpp"

#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <thread>

namespace rebot_arm_rs_hardware
{
namespace
{
void checked(bool success, const char * operation)
{
  if (!success) {
    throw std::system_error(errno, std::generic_category(), operation);
  }
}
}  // namespace

SocketCan::~SocketCan() {close();}

void SocketCan::open(const std::string & interface, uint8_t host_id)
{
  close();
  if (interface.empty() || interface.size() >= IFNAMSIZ) {
    throw std::invalid_argument("Invalid SocketCAN interface name");
  }
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  checked(fd_ >= 0, "socket(PF_CAN)");
  try {
    ifreq request{};
    std::memcpy(request.ifr_name, interface.c_str(), interface.size() + 1);
    checked(::ioctl(fd_, SIOCGIFINDEX, &request) == 0, "CAN interface lookup");
    const int index = request.ifr_ifindex;
    checked(::ioctl(fd_, SIOCGIFFLAGS, &request) == 0, "CAN interface flags");
    if (!(request.ifr_flags & IFF_UP)) {
      throw std::runtime_error("SocketCAN interface " + interface + " is DOWN");
    }
    can_filter filter{};
    filter.can_id = CAN_EFF_FLAG | host_id;
    filter.can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | 0xFF;
    checked(::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) == 0,
      "CAN receive filter");
    const can_err_mask_t errors = CAN_ERR_BUSOFF | CAN_ERR_RESTARTED | CAN_ERR_CRTL;
    checked(::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) == 0,
      "CAN error filter");
    const int receive_own = 0;
    checked(::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
      &receive_own, sizeof(receive_own)) == 0, "CAN own-frame filter");
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = index;
    checked(::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0,
      "bind(SocketCAN)");
  } catch (...) {
    close();
    throw;
  }
}

void SocketCan::close() noexcept
{
  if (fd_ >= 0) {::close(fd_);}
  fd_ = -1;
  last_sent_ = {};
}

ssize_t SocketCan::transmit(const can_frame & frame)
{
  if (fd_ < 0) {throw std::runtime_error("CAN socket is not open");}
  return ::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
}

void SocketCan::send(const can_frame & frame)
{
  using Clock = std::chrono::steady_clock;
  // Avoid pushing a whole seven-motor query/activation burst into the default
  // ten-frame netdevice queue. This driver uses classic extended CAN at 1 Mbps.
  constexpr auto spacing = std::chrono::microseconds(200);
  constexpr auto timeout = std::chrono::milliseconds(5);
  if (last_sent_ != Clock::time_point{}) {std::this_thread::sleep_until(last_sent_ + spacing);}
  const auto deadline = Clock::now() + timeout;
  while (true) {
    const auto count = transmit(frame);
    if (count == static_cast<ssize_t>(sizeof(frame))) {
      last_sent_ = Clock::now();
      return;  // Never replay a frame once the kernel has accepted it.
    }
    if (count >= 0) {throw std::runtime_error("SocketCAN send: incomplete classic CAN frame");}
    const int error = errno;
    const bool transient = error == ENOBUFS || error == EAGAIN ||
      error == EWOULDBLOCK || error == EINTR;
    if (!transient) {throw std::system_error(error, std::generic_category(), "SocketCAN send");}
    if (Clock::now() >= deadline) {
      throw std::system_error(error, std::generic_category(), "SocketCAN TX did not recover within 5 ms");
    }
    // POLLOUT may stay asserted when the netdevice queue (rather than the
    // socket buffer) is full. Back off instead of spinning on poll().
    std::this_thread::sleep_for(spacing);
    if (Clock::now() >= deadline) {
      throw std::system_error(error, std::generic_category(), "SocketCAN TX did not recover within 5 ms");
    }
  }
}

bool SocketCan::receive(can_frame & frame)
{
  if (fd_ < 0) {throw std::runtime_error("CAN socket is not open");}
  const auto count = ::recv(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
  if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
    return false;
  }
  checked(count >= 0, "SocketCAN receive");
  if (count != static_cast<ssize_t>(sizeof(frame))) {throw std::runtime_error("Invalid classic CAN frame length");}
  if (frame.can_id & CAN_ERR_FLAG) {
    throw std::runtime_error("SocketCAN bus/controller error, CAN error ID=" +
      std::to_string(frame.can_id & CAN_ERR_MASK));
  }
  return true;
}
}  // namespace rebot_arm_rs_hardware
