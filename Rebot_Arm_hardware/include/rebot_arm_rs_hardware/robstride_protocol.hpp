#pragma once

#include <linux/can.h>
#include <array>
#include <cstdint>
#include <optional>

namespace rebot_arm_rs_hardware::protocol
{
constexpr uint16_t kRunMode = 0x7005;
constexpr uint16_t kPositionTarget = 0x7016;
constexpr uint16_t kSpeedLimit = 0x7017;
constexpr uint16_t kCurrentLimit = 0x7018;
constexpr uint16_t kPosition = 0x7019;
constexpr uint16_t kVelocity = 0x701B;
constexpr uint16_t kCanTimeout = 0x7028;
constexpr uint8_t kCspMode = 5;

struct Reply
{
  uint8_t type{};
  uint8_t motor_id{};
  uint8_t flags{};  // Type 17: read error; type 2/24: fault bits and mode.
  uint16_t index{};
  std::array<uint8_t, 4> value{};
  uint32_t fault{};
  uint32_t warning{};
  float as_float() const;
  uint32_t as_u32() const;
};

can_frame read_parameter(uint8_t host, uint8_t motor, uint16_t index);
can_frame write_float(uint8_t host, uint8_t motor, uint16_t index, float value);
can_frame write_u32(uint8_t host, uint8_t motor, uint16_t index, uint32_t value);
can_frame write_u8(uint8_t host, uint8_t motor, uint16_t index, uint8_t value);
can_frame enable(uint8_t host, uint8_t motor);
can_frame stop(uint8_t host, uint8_t motor);
std::optional<Reply> decode(const can_frame & frame, uint8_t host);
}  // namespace rebot_arm_rs_hardware::protocol
