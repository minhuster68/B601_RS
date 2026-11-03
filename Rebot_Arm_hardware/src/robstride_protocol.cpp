#include "rebot_arm_rs_hardware/robstride_protocol.hpp"

#include <cmath>
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace rebot_arm_rs_hardware::protocol
{
namespace
{
can_frame make_frame(uint8_t type, uint8_t host, uint8_t motor, uint16_t index = 0)
{
  if (motor == 0 || host == 0 || motor == host) {
    throw std::invalid_argument("CAN IDs must be nonzero and motor must differ from host");
  }
  can_frame frame{};
  frame.can_id = CAN_EFF_FLAG | (uint32_t(type) << 24) | (uint32_t(host) << 8) | motor;
  frame.can_dlc = 8;
  frame.data[0] = index & 0xFF;
  frame.data[1] = index >> 8;
  return frame;
}

uint32_t u32(const uint8_t * data)
{
  return uint32_t(data[0]) | (uint32_t(data[1]) << 8) |
         (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
}
}  // namespace

can_frame read_parameter(uint8_t host, uint8_t motor, uint16_t index)
{
  return make_frame(17, host, motor, index);
}

can_frame write_u32(uint8_t host, uint8_t motor, uint16_t index, uint32_t value)
{
  auto frame = make_frame(18, host, motor, index);
  for (unsigned i = 0; i < 4; ++i) {
    frame.data[4 + i] = (value >> (8 * i)) & 0xFF;
  }
  return frame;
}

can_frame write_float(uint8_t host, uint8_t motor, uint16_t index, float value)
{
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  if (!std::isfinite(value)) {
    throw std::invalid_argument("Refusing to encode a non-finite motor command");
  }
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return write_u32(host, motor, index, bits);
}

can_frame write_u8(uint8_t host, uint8_t motor, uint16_t index, uint8_t value)
{
  return write_u32(host, motor, index, value);
}

can_frame enable(uint8_t host, uint8_t motor) {return make_frame(3, host, motor);}
can_frame stop(uint8_t host, uint8_t motor) {return make_frame(4, host, motor);}

uint32_t Reply::as_u32() const {return u32(value.data());}

float Reply::as_float() const
{
  const uint32_t bits = as_u32();
  float result;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

std::optional<Reply> decode(const can_frame & frame, uint8_t host)
{
  if (!(frame.can_id & CAN_EFF_FLAG) || (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) ||
    frame.can_dlc != 8 || (frame.can_id & 0xFF) != host)
  {
    return std::nullopt;
  }
  Reply result;
  result.type = (frame.can_id >> 24) & 0x1F;
  result.motor_id = (frame.can_id >> 8) & 0xFF;
  result.flags = (frame.can_id >> 16) & 0xFF;
  if (result.type == 17) {
    if (frame.data[2] != 0 || frame.data[3] != 0) {
      return std::nullopt;
    }
    result.index = uint16_t(frame.data[0]) | (uint16_t(frame.data[1]) << 8);
    std::copy(frame.data + 4, frame.data + 8, result.value.begin());
  } else if (result.type == 21) {
    result.fault = u32(frame.data);
    result.warning = u32(frame.data + 4);
  } else if (result.type != 2 && result.type != 24) {
    return std::nullopt;
  }
  return result;
}
}  // namespace rebot_arm_rs_hardware::protocol
