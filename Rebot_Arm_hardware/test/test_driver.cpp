#include "rebot_arm_rs_hardware/robstride_protocol.hpp"
#include "rebot_arm_rs_hardware/rs_system.hpp"

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <system_error>

namespace hw = rebot_arm_rs_hardware;
namespace p = rebot_arm_rs_hardware::protocol;
using Return = hardware_interface::return_type;
using Callback = hw::RSSystem::CallbackReturn;

namespace
{
uint32_t bits(float value)
{
  uint32_t result;
  std::memcpy(&result, &value, 4);
  return result;
}

can_frame reply(uint8_t motor, uint16_t index, uint32_t value, uint8_t flags = 0)
{
  auto frame = p::write_u32(253, motor, index, value);
  frame.can_id = CAN_EFF_FLAG | (17u << 24) | (uint32_t(flags) << 16) | (uint32_t(motor) << 8) | 253;
  return frame;
}

can_frame status(uint8_t motor, uint8_t flags)
{
  can_frame frame{};
  frame.can_id = CAN_EFF_FLAG | (2u << 24) | (uint32_t(flags) << 16) | (uint32_t(motor) << 8) | 253;
  frame.can_dlc = 8;
  return frame;
}

struct FakeMotor
{
  float position{1.5}, velocity{0};
  bool running{false};
  std::map<uint16_t, uint32_t> registers;
};

struct FakeBus
{
  std::map<uint8_t, FakeMotor> motors{{1, {}}, {2, {}}};
  std::deque<can_frame> queue;
  std::vector<can_frame> sent;
  bool position_replies{true}, velocity_replies{true};
  bool open{false}, fail_send{false}, follow_target{false};
  std::optional<uint16_t> ignore_register;
  std::optional<uint8_t> ignore_enable;
};

class FakeTransport : public hw::CanTransport
{
public:
  explicit FakeTransport(std::shared_ptr<FakeBus> bus) : bus_(std::move(bus)) {}
  void open(const std::string &, uint8_t host) override
  {
    EXPECT_EQ(host, 253);
    bus_->open = true;
  }
  void close() noexcept override {bus_->open = false;}
  void send(const can_frame & frame) override
  {
    if (bus_->fail_send) {throw std::runtime_error("Injected CAN send failure");}
    bus_->sent.push_back(frame);
    const auto id = static_cast<uint8_t>(frame.can_id & 0xFF);
    const auto type = (frame.can_id >> 24) & 0x1F;
    if (!bus_->motors.count(id)) {return;}
    auto & motor = bus_->motors.at(id);
    const uint16_t index = frame.data[0] | (uint16_t(frame.data[1]) << 8);
    if (type == 17) {
      if ((index == p::kPosition && !bus_->position_replies) ||
        (index == p::kVelocity && !bus_->velocity_replies)) {return;}
      const uint32_t value = index == p::kPosition ? bits(motor.position) :
        index == p::kVelocity ? bits(motor.velocity) : motor.registers[index];
      bus_->queue.push_back(reply(id, index, value));
    } else if (type == 18) {
      const uint32_t value = frame.data[4] | (uint32_t(frame.data[5]) << 8) |
        (uint32_t(frame.data[6]) << 16) | (uint32_t(frame.data[7]) << 24);
      if (!bus_->ignore_register || *bus_->ignore_register != index) {motor.registers[index] = value;}
      if (bus_->follow_target && index == p::kPositionTarget) {
        std::memcpy(&motor.position, &value, 4);
      }
      bus_->queue.push_back(status(id, motor.running ? 0x80 : 0));
    } else if (type == 3) {
      if (bus_->ignore_enable && *bus_->ignore_enable == id) {return;}
      motor.running = true;
      bus_->queue.push_back(status(id, 0x80));
    } else if (type == 4) {
      motor.running = false;
      bus_->queue.push_back(status(id, 0));
    }
  }
  bool receive(can_frame & frame) override
  {
    if (bus_->queue.empty()) {return false;}
    frame = bus_->queue.front();
    bus_->queue.pop_front();
    return true;
  }
private:
  std::shared_ptr<FakeBus> bus_;
};

class TestSystem : public hw::RSSystem
{
public:
  std::shared_ptr<FakeBus> fake = std::make_shared<FakeBus>();
  std::chrono::milliseconds offset{};
protected:
  std::unique_ptr<hw::CanTransport> make_transport() override
  {
    return std::make_unique<FakeTransport>(fake);
  }
  Clock::time_point now() const override {return Clock::now() + offset;}
};

class CongestedSocket : public hw::SocketCan
{
public:
  std::deque<int> errors;
  std::vector<can_frame> attempts, accepted;
  std::vector<std::chrono::steady_clock::time_point> accepted_at;
  int persistent_error{0};
  bool short_send{false};
protected:
  ssize_t transmit(const can_frame & frame) override
  {
    attempts.push_back(frame);
    int error = persistent_error;
    if (!errors.empty()) {error = errors.front(); errors.pop_front();}
    if (error) {errno = error; return -1;}
    if (short_send) {return sizeof(frame) - 1;}
    accepted.push_back(frame);
    accepted_at.push_back(std::chrono::steady_clock::now());
    return sizeof(frame);
  }
};

hardware_interface::InterfaceInfo interface(const std::string & name)
{
  hardware_interface::InterfaceInfo result;
  result.name = name;
  return result;
}

hardware_interface::HardwareInfo info(bool read_only, int count = 2)
{
  hardware_interface::HardwareInfo result;
  result.name = "test_rs_system";
  result.type = "system";
  result.hardware_parameters = {
    {"can_interface", "not_a_real_can_interface"}, {"host_id", "253"},
    {"read_only", read_only ? "true" : "false"}, {"calibration_confirmed", "true"},
    {"feedback_timeout_s", "0.15"}, {"startup_timeout_s", "0.03"},
    {"poll_rate_hz", "50"}, {"max_control_period_s", "0.05"}, {"watchdog_ms", "200"}};
  for (int i = 1; i <= count; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(i);
    joint.type = "joint";
    joint.parameters = {
      {"motor_id", std::to_string(i)}, {"direction", "1"}, {"motor_zero_rad", "1.0"},
      {"joint_zero", "0"}, {"motor_radians_per_joint_unit", "1"},
      {"position_min", "-2"}, {"position_max", "2"}, {"max_velocity", "0.2"},
      {"max_tracking_error", "0.15"}, {"current_limit_a", "1.0"}};
    joint.state_interfaces = {interface("position"), interface("velocity")};
    if (!read_only) {joint.command_interfaces = {interface("position")};}
    result.joints.push_back(joint);
  }
  hardware_interface::ComponentInfo mimic;
  mimic.name = "follower";
  mimic.parameters = {{"mimic", "joint1"}, {"multiplier", "1"}, {"offset", "0"}};
  mimic.state_interfaces = {interface("position"), interface("velocity")};
  result.joints.push_back(mimic);
  return result;
}

void activate(TestSystem & system, bool read_only = false)
{
  ASSERT_EQ(system.on_init(info(read_only)), Callback::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
}

bool is_type(const can_frame & frame, unsigned type) {return ((frame.can_id >> 24) & 0x1F) == type;}

bool is_target(const can_frame & frame)
{
  return is_type(frame, 18) && (frame.data[0] | (uint16_t(frame.data[1]) << 8)) == p::kPositionTarget;
}

float target(const can_frame & frame)
{
  auto feedback = reply(1, p::kPositionTarget, 0);
  std::copy(frame.data + 4, frame.data + 8, feedback.data + 4);
  return p::decode(feedback, 253)->as_float();
}
}  // namespace

TEST(Protocol, VendorReadExampleAndKnownFloatBytes)
{
  const auto frame = p::read_parameter(0xFD, 0x7F, 0x701E);
  EXPECT_EQ(frame.can_id, CAN_EFF_FLAG | 0x1100FD7Fu);
  const std::array<uint8_t, 8> bytes{0x1E, 0x70, 0, 0, 0, 0, 0, 0};
  EXPECT_EQ(std::memcmp(frame.data, bytes.data(), 8), 0);
  const auto command = p::write_float(253, 1, p::kPositionTarget, 1.25f);
  EXPECT_EQ(command.can_id, CAN_EFF_FLAG | 0x1200FD01u);
  const std::array<uint8_t, 8> position{0x16, 0x70, 0, 0, 0, 0, 0xA0, 0x3F};
  EXPECT_EQ(std::memcmp(command.data, position.data(), 8), 0);
  const auto mode = p::write_u8(253, 1, p::kRunMode, 5);
  EXPECT_EQ(mode.data[0], 0x05);
  EXPECT_EQ(mode.data[4], 5);
  const auto watchdog = p::write_u32(253, 1, p::kCanTimeout, 4000);
  EXPECT_EQ(watchdog.data[4], 0xA0);
  EXPECT_EQ(watchdog.data[5], 0x0F);
}

TEST(Protocol, PreservesMultiturnPositionAndAllStatusFlags)
{
  const auto value = p::decode(reply(7, p::kPosition, bits(100.25f)), 253);
  ASSERT_TRUE(value);
  EXPECT_EQ(value->motor_id, 7);
  EXPECT_FLOAT_EQ(value->as_float(), 100.25f);
  const auto state = p::decode(status(7, 0xE1), 253);
  ASSERT_TRUE(state);
  EXPECT_EQ(state->flags & 0x3F, 0x21);
  EXPECT_EQ(state->flags >> 6, 3);
}

TEST(Protocol, RejectsWrongAddressAndMalformedFrames)
{
  auto frame = reply(1, p::kPosition, bits(0));
  EXPECT_FALSE(p::decode(frame, 252));
  frame.can_dlc = 7;
  EXPECT_FALSE(p::decode(frame, 253));
  frame.can_dlc = 8;
  frame.can_id &= ~CAN_EFF_FLAG;
  EXPECT_FALSE(p::decode(frame, 253));
  frame.can_id |= CAN_EFF_FLAG | CAN_RTR_FLAG;
  EXPECT_FALSE(p::decode(frame, 253));
  EXPECT_THROW(p::write_float(253, 1, p::kPositionTarget,
    std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(p::read_parameter(253, 253, p::kPosition), std::invalid_argument);
}

TEST(Driver, ReadOnlyNeverEnablesDisablesOrWritesParameters)
{
  TestSystem system;
  activate(system, true);
  EXPECT_TRUE(system.export_command_interfaces().empty());
  auto states = system.export_state_interfaces();
  EXPECT_DOUBLE_EQ(states[0].get_value(), 0.5);
  EXPECT_DOUBLE_EQ(states[4].get_value(), 0.5);  // Mimic of joint1.
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::OK);
  system.on_deactivate(rclcpp_lifecycle::State{});
  system.on_cleanup(rclcpp_lifecycle::State{});
  for (const auto & frame : system.fake->sent) {EXPECT_TRUE(is_type(frame, 17));}
}

TEST(Driver, StartsAtMeasuredPositionAndVerifiesFirmwareWatchdog)
{
  TestSystem system;
  activate(system);
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(commands.size(), 2u);
  EXPECT_DOUBLE_EQ(commands[0].get_value(), 0.5);
  EXPECT_EQ(system.fake->motors[1].registers[p::kCanTimeout], 4000u);
  EXPECT_EQ(system.fake->motors[1].registers[p::kRunMode], 5u);
  for (const auto & frame : system.fake->sent) {
    if (is_target(frame)) {EXPECT_FLOAT_EQ(target(frame), 1.5f);}
  }
  system.on_deactivate(rclcpp_lifecycle::State{});
  EXPECT_FALSE(system.fake->motors[1].running);
  EXPECT_FALSE(system.fake->motors[2].running);
}

TEST(Driver, MotorScaleDirectionAndPrismaticConversionAreConsistent)
{
  TestSystem system;
  auto configuration = info(false, 1);
  auto & parameters = configuration.joints[0].parameters;
  parameters["direction"] = "-1";
  parameters["motor_radians_per_joint_unit"] = "100";
  parameters["joint_zero"] = "0.01";
  parameters["max_velocity"] = "0.01";
  system.fake->motors[1].velocity = 0.2f;
  ASSERT_EQ(system.on_init(configuration), Callback::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  auto states = system.export_state_interfaces();
  EXPECT_NEAR(states[0].get_value(), 0.005, 1e-9);
  EXPECT_NEAR(states[1].get_value(), -0.002, 1e-9);
  auto commands = system.export_command_interfaces();
  commands[0].set_value(0.00505);
  system.fake->sent.clear();
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::OK);
  ASSERT_EQ(system.fake->sent.size(), 1u);
  EXPECT_NEAR(target(system.fake->sent[0]), 1.495, 1e-6);
}

TEST(Driver, LimitsPositionTargetSlew)
{
  TestSystem system;
  activate(system);
  auto commands = system.export_command_interfaces();
  commands[0].set_value(1.5);
  system.fake->sent.clear();
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::OK);
  EXPECT_NEAR(target(system.fake->sent[0]), 1.502f, 1e-6);
}

TEST(Driver, FeedbackAtToleranceIsBoundedAndInitialTargetsStayInsideLimits)
{
  for (const float measured : {-0.05f, 1.05f}) {
    TestSystem system;
    auto configuration = info(false, 1);
    configuration.hardware_parameters["position_limit_tolerance_rad"] = "0.05";
    auto & params = configuration.joints[0].parameters;
    params["motor_zero_rad"] = "0";
    params["position_min"] = "0";
    params["position_max"] = "1";
    system.fake->motors[1].position = measured;
    ASSERT_EQ(system.on_init(configuration), Callback::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    const double boundary = measured < 0 ? 0.0 : 1.0;
    EXPECT_DOUBLE_EQ(system.export_state_interfaces()[0].get_value(), boundary);
    EXPECT_DOUBLE_EQ(system.export_state_interfaces()[2].get_value(), boundary);  // Mimic.
    EXPECT_DOUBLE_EQ(system.export_command_interfaces()[0].get_value(), boundary);
    for (const auto & frame : system.fake->sent) {
      if (is_target(frame)) {EXPECT_FLOAT_EQ(target(frame), float(boundary));}
    }
    EXPECT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::OK);
  }
}

TEST(Driver, MonitorAcceptsSmallOvertravelButRejectsBeyondToleranceWithoutWrites)
{
  for (const float measured : {-0.049f, -0.0501f, 1.049f, 1.0501f}) {
    TestSystem system;
    auto configuration = info(true, 1);
    auto & params = configuration.joints[0].parameters;
    params["motor_zero_rad"] = "0";
    params["position_min"] = "0";
    params["position_max"] = "1";
    system.fake->motors[1].position = measured;
    ASSERT_EQ(system.on_init(configuration), Callback::SUCCESS);
    const bool allowed = measured >= -0.05f && measured <= 1.05f;
    EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
      allowed ? Callback::SUCCESS : Callback::ERROR);
    if (allowed) {
      EXPECT_DOUBLE_EQ(system.export_state_interfaces()[0].get_value(), measured < 0 ? 0.0 : 1.0);
    }
    for (const auto & frame : system.fake->sent) {EXPECT_TRUE(is_type(frame, 17));}
  }
}

TEST(Driver, GripperToleranceUsesMotorRadiansWithScaleAndDirection)
{
  for (const float measured : {0.05f, 0.0501f}) {
    TestSystem system;
    auto configuration = info(true, 1);
    auto & params = configuration.joints[0].parameters;
    params["motor_zero_rad"] = "0";
    params["direction"] = "-1";
    params["motor_radians_per_joint_unit"] = "119.2602";
    params["max_velocity"] = "0.01";
    params["position_min"] = "0";
    params["position_max"] = "0.05";
    system.fake->motors[1].position = measured;
    ASSERT_EQ(system.on_init(configuration), Callback::SUCCESS);
    EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
      measured == 0.05f ? Callback::SUCCESS : Callback::ERROR);
    if (measured == 0.05f) {EXPECT_DOUBLE_EQ(system.export_state_interfaces()[0].get_value(), 0);}
  }
}

TEST(Driver, OvertravelFaultStopsOwnedMotorsAndLatches)
{
  TestSystem system;
  activate(system);
  system.fake->queue.clear();
  system.fake->queue.push_back(reply(1, p::kPosition, bits(-1.0501f)));  // q < -2 - 0.05.
  system.fake->sent.clear();
  EXPECT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
  EXPECT_FALSE(system.fake->motors[1].running);
  EXPECT_FALSE(system.fake->motors[2].running);
  ASSERT_EQ(system.fake->sent.size(), 2u);
  for (const auto & frame : system.fake->sent) {EXPECT_TRUE(is_type(frame, 4));}
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
  EXPECT_EQ(system.fake->sent.size(), 2u);
}

TEST(Driver, BoundedFeedbackDoesNotHideRawTrackingErrorOrAllowOutOfRangeCommands)
{
  for (const bool invalid_command : {false, true}) {
    TestSystem system;
    auto configuration = info(false, 1);
    auto & params = configuration.joints[0].parameters;
    params["motor_zero_rad"] = "0";
    params["position_min"] = "0";
    params["position_max"] = "1";
    params["max_tracking_error"] = "0.01";
    system.fake->motors[1].position = invalid_command ? 0.0f : -0.02f;
    ASSERT_EQ(system.on_init(configuration), Callback::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    EXPECT_DOUBLE_EQ(system.export_state_interfaces()[0].get_value(), 0);
    system.fake->sent.clear();
    if (invalid_command) {
      system.export_command_interfaces()[0].set_value(-0.001);
      EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
    } else {
      EXPECT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
    }
    EXPECT_FALSE(system.fake->motors[1].running);
    for (const auto & frame : system.fake->sent) {EXPECT_TRUE(is_type(frame, 4));}
  }
}

TEST(Driver, RejectsInvalidPositionToleranceBeforeOpeningBus)
{
  for (const std::string tolerance : {"-0.05", "nan", "inf"}) {
    TestSystem system;
    auto configuration = info(true, 1);
    configuration.hardware_parameters["position_limit_tolerance_rad"] = tolerance;
    EXPECT_EQ(system.on_init(configuration), Callback::ERROR);
    EXPECT_FALSE(system.fake->open);
  }
}

TEST(Driver, InvalidCommandStopsAllMotorsBeforeSendingAnyNewTarget)
{
  TestSystem system;
  activate(system);
  auto commands = system.export_command_interfaces();
  commands[0].set_value(0.6);
  commands[1].set_value(std::numeric_limits<double>::quiet_NaN());
  system.fake->sent.clear();
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
  ASSERT_EQ(system.fake->sent.size(), 2u);
  for (const auto & frame : system.fake->sent) {EXPECT_TRUE(is_type(frame, 4));}
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
  EXPECT_EQ(system.fake->sent.size(), 2u);  // No automatic re-enable.
}

TEST(Driver, StaleVelocityFailsEvenWhenPositionIsFresh)
{
  TestSystem system;
  activate(system);
  system.fake->queue.clear();
  system.fake->velocity_replies = false;
  system.offset = std::chrono::milliseconds(180);
  EXPECT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
  EXPECT_FALSE(system.fake->motors[1].running);
  EXPECT_FALSE(system.fake->motors[2].running);
}

TEST(Driver, AllSixFaultBitsAreHandledAndResetModeDoesNotAutoEnable)
{
  for (const auto flags : {uint8_t(0xA0), uint8_t(0x00)}) {
    TestSystem system;
    activate(system);
    system.fake->queue.clear();
    system.fake->queue.push_back(status(1, flags));
    EXPECT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
    EXPECT_FALSE(system.fake->motors[1].running);
    EXPECT_FALSE(system.fake->motors[2].running);
  }
}

TEST(Driver, ReadErrorAndNonFiniteFeedbackLatchFaults)
{
  for (const auto bad_reply : {
    reply(1, p::kPosition, bits(0), 1),
    reply(1, p::kPosition, bits(std::numeric_limits<float>::infinity()))})
  {
    TestSystem system;
    activate(system);
    system.fake->queue.clear();
    system.fake->queue.push_back(bad_reply);
    EXPECT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
    EXPECT_FALSE(system.fake->motors[1].running);
  }
}

TEST(Driver, RejectsLongControlCycleAndSendFailure)
{
  TestSystem system;
  activate(system);
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::OK);
  system.offset = std::chrono::milliseconds(100);
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.1)), Return::ERROR);
  EXPECT_FALSE(system.fake->motors[1].running);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  system.fake->fail_send = true;
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
}

TEST(Driver, FirstWriteExcludesStartupTimeButLaterLongPeriodsStillFault)
{
  TestSystem system;
  activate(system);
  system.fake->sent.clear();
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.5)), Return::OK);
  for (const auto & frame : system.fake->sent) {
    ASSERT_TRUE(is_target(frame));
    EXPECT_FLOAT_EQ(target(frame), 1.5f);  // Startup period cannot cause a target jump.
  }
  system.offset = std::chrono::milliseconds(100);
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.1)), Return::ERROR);
  EXPECT_FALSE(system.fake->motors[1].running);
}

TEST(Driver, DelayedFirstWriteStillFaultsAfterActivation)
{
  TestSystem system;
  activate(system);
  system.offset = std::chrono::milliseconds(60);
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::ERROR);
  EXPECT_FALSE(system.fake->motors[1].running);
}

TEST(Driver, StartupTimeReportedOnSecondWriteStillHoldsWithoutAdvancingTargets)
{
  TestSystem system;
  activate(system);
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), Return::OK);
  system.export_command_interfaces()[0].set_value(1.0);
  system.fake->sent.clear();
  EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.1)), Return::OK);
  for (const auto & frame : system.fake->sent) {
    ASSERT_TRUE(is_target(frame));
    EXPECT_FLOAT_EQ(target(frame), 1.5f);
  }
}

TEST(Driver, WatchdogReadbackFailurePreventsAllEnableCommands)
{
  TestSystem system;
  ASSERT_EQ(system.on_init(info(false)), Callback::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  system.fake->ignore_register = p::kCanTimeout;
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State{}), Callback::ERROR);
  for (const auto & frame : system.fake->sent) {EXPECT_FALSE(is_type(frame, 3));}
}

TEST(Driver, PartialEnableFailureStopsPreviouslyEnabledMotors)
{
  TestSystem system;
  ASSERT_EQ(system.on_init(info(false)), Callback::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  system.fake->ignore_enable = 2;
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State{}), Callback::ERROR);
  EXPECT_FALSE(system.fake->motors[1].running);
  EXPECT_FALSE(system.fake->motors[2].running);
}

TEST(Driver, MissingMotorFailsConfigurationWithoutAnyEnable)
{
  TestSystem system;
  system.fake->motors.erase(2);
  ASSERT_EQ(system.on_init(info(false)), Callback::SUCCESS);
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}), Callback::ERROR);
  EXPECT_FALSE(system.fake->open);
  for (const auto & frame : system.fake->sent) {EXPECT_TRUE(is_type(frame, 17));}
}

TEST(Driver, RejectsBadMappingAndUnconfirmedCalibrationBeforeOpeningBus)
{
  TestSystem system;
  auto configuration = info(false);
  configuration.hardware_parameters["calibration_confirmed"] = "false";
  EXPECT_EQ(system.on_init(configuration), Callback::ERROR);
  configuration = info(false);
  configuration.joints[1].parameters["motor_id"] = "1";
  EXPECT_EQ(system.on_init(configuration), Callback::ERROR);
  configuration = info(false);
  configuration.joints[0].parameters["direction"] = "0";
  EXPECT_EQ(system.on_init(configuration), Callback::ERROR);
  EXPECT_FALSE(system.fake->open);
}

TEST(Plugin, LoadsUsingRos2ControlPluginlibName)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  EXPECT_NO_THROW(loader.createSharedInstance("rebot_arm_rs_hardware/RSSystem"));
}

TEST(SocketCan, MissingInterfaceFailsWithoutTouchingRealCan0)
{
  hw::SocketCan socket;
  EXPECT_THROW(socket.open("no_such_can", 253), std::exception);
  EXPECT_THROW(socket.open(std::string(30, 'x'), 253), std::invalid_argument);
}

TEST(SocketCan, TransientCongestionRetriesOnlyUnacceptedFrameAndKeepsOrder)
{
  CongestedSocket socket;
  const auto first = p::read_parameter(253, 1, p::kPosition);
  const auto second = p::write_float(253, 2, p::kPositionTarget, 1.0f);
  socket.errors = {ENOBUFS, EAGAIN, EINTR, 0};
  EXPECT_NO_THROW(socket.send(first));
  EXPECT_NO_THROW(socket.send(second));
  ASSERT_EQ(socket.attempts.size(), 5u);
  ASSERT_EQ(socket.accepted.size(), 2u);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(std::memcmp(&socket.attempts[i], &first, sizeof(first)), 0);
  }
  EXPECT_EQ(std::memcmp(&socket.accepted[0], &first, sizeof(first)), 0);
  EXPECT_EQ(std::memcmp(&socket.accepted[1], &second, sizeof(second)), 0);
  EXPECT_GE(socket.accepted_at[1] - socket.accepted_at[0], std::chrono::microseconds(200));
}

TEST(SocketCan, PermanentCongestionTimesOutAndNeverAcceptsOrReordersCommands)
{
  CongestedSocket socket;
  socket.persistent_error = ENOBUFS;
  const auto start = std::chrono::steady_clock::now();
  try {
    socket.send(p::write_float(253, 1, p::kPositionTarget, 1.0f));
    FAIL() << "Persistent congestion must fail";
  } catch (const std::system_error & error) {
    EXPECT_EQ(error.code().value(), ENOBUFS);
    EXPECT_NE(std::string(error.what()).find("within 5 ms"), std::string::npos);
  }
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(5));
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(250));
  EXPECT_TRUE(socket.accepted.empty());
}

TEST(SocketCan, PermanentIoErrorsAndShortFramesFailWithoutRetry)
{
  CongestedSocket socket;
  socket.persistent_error = ENETDOWN;
  EXPECT_THROW(socket.send(p::read_parameter(253, 1, p::kPosition)), std::system_error);
  EXPECT_EQ(socket.attempts.size(), 1u);
  socket.persistent_error = 0;
  socket.short_send = true;
  EXPECT_THROW(socket.send(p::read_parameter(253, 1, p::kPosition)), std::runtime_error);
  EXPECT_EQ(socket.attempts.size(), 2u);
  EXPECT_TRUE(socket.accepted.empty());
}
