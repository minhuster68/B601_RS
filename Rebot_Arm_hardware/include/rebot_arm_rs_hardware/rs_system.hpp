#pragma once

#include "rebot_arm_rs_hardware/robstride_protocol.hpp"
#include "rebot_arm_rs_hardware/socket_can.hpp"

#include <hardware_interface/system_interface.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rebot_arm_rs_hardware
{
class RSSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Clock = std::chrono::steady_clock;
  ~RSSystem() override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  virtual std::unique_ptr<CanTransport> make_transport();
  virtual Clock::time_point now() const;

private:
  struct Motor
  {
    std::size_t joint{};
    uint8_t id{};
    double direction{}, motor_zero{}, joint_zero{}, scale{};
    double lower{}, upper{}, velocity{}, tracking_error{}, current{};
    double motor_position{}, last_sent{};
    Clock::time_point position_at{}, velocity_at{}, status_at{}, clamp_warning_at{};
    uint8_t mode{};
    bool owned{false}, position_clamped{false};
    double to_joint(double angle) const;
    double to_motor(double position) const;
  };
  struct Mimic {std::size_t joint{}, parent{}; double factor{1}, offset{0};};
  struct Pending {uint8_t motor{}; uint16_t index{}; std::optional<protocol::Reply> reply;};

  void receive();
  void query_states();
  bool fresh() const;
  void wait_for_states();
  void verify(const Motor &, const can_frame &, uint16_t index, uint32_t expected);
  void stop_owned() noexcept;
  void close() noexcept;
  void fail(const std::string & reason) noexcept;
  void update_mimics();

  std::unique_ptr<CanTransport> bus_;
  std::vector<Motor> motors_;
  std::vector<Mimic> mimics_;
  std::vector<double> positions_, velocities_, commands_;
  std::optional<Pending> pending_;
  std::string interface_;
  uint8_t host_{253};
  bool read_only_{true}, calibrated_{false}, active_{false}, faulted_{false};
  bool first_write_{false};
  double feedback_timeout_{0.15}, startup_timeout_{2}, poll_period_{0.02};
  double max_control_period_{0.05};
  double position_limit_tolerance_rad_{0.05};
  uint32_t watchdog_ticks_{4000};
  Clock::time_point last_poll_{}, activated_at_{};
};
}  // namespace rebot_arm_rs_hardware
