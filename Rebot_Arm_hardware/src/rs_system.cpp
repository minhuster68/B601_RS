#include "rebot_arm_rs_hardware/rs_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>

namespace rebot_arm_rs_hardware
{
namespace
{
const auto logger = rclcpp::get_logger("rebot_arm_rs_hardware");
using Parameters = std::unordered_map<std::string, std::string>;

std::string required(const Parameters & parameters, const std::string & name)
{
  const auto entry = parameters.find(name);
  if (entry == parameters.end() || entry->second.empty()) {
    throw std::invalid_argument("Missing hardware parameter: " + name);
  }
  return entry->second;
}

double number(const Parameters & parameters, const std::string & name)
{
  const auto text = required(parameters, name);
  std::size_t count = 0;
  const auto result = std::stod(text, &count);
  if (count != text.size() || !std::isfinite(result)) {
    throw std::invalid_argument("Invalid finite number for " + name);
  }
  return result;
}

bool boolean(const Parameters & parameters, const std::string & name)
{
  const auto text = required(parameters, name);
  if (text == "true") {return true;}
  if (text == "false") {return false;}
  throw std::invalid_argument(name + " must be true or false");
}

uint8_t id(const Parameters & parameters, const std::string & name)
{
  const auto result = number(parameters, name);
  if (result < 1 || result > 254 || std::floor(result) != result) {
    throw std::invalid_argument(name + " must be an integer in [1,254]");
  }
  return static_cast<uint8_t>(result);
}

bool has_interfaces(const std::vector<hardware_interface::InterfaceInfo> & interfaces)
{
  std::set<std::string> names;
  for (const auto & interface : interfaces) {names.insert(interface.name);}
  return names == std::set<std::string>{"position", "velocity"} && interfaces.size() == 2;
}
}  // namespace

double RSSystem::Motor::to_joint(double angle) const
{
  return joint_zero + direction * (angle - motor_zero) / scale;
}

double RSSystem::Motor::to_motor(double position) const
{
  return motor_zero + direction * scale * (position - joint_zero);
}

RSSystem::~RSSystem() {close();}
RSSystem::Clock::time_point RSSystem::now() const {return Clock::now();}
std::unique_ptr<CanTransport> RSSystem::make_transport() {return std::make_unique<SocketCan>();}

RSSystem::CallbackReturn RSSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {return CallbackReturn::ERROR;}
  try {
    close();
    motors_.clear();
    mimics_.clear();
    const auto & parameters = info_.hardware_parameters;
    interface_ = required(parameters, "can_interface");
    host_ = id(parameters, "host_id");
    read_only_ = boolean(parameters, "read_only");
    calibrated_ = boolean(parameters, "calibration_confirmed");
    feedback_timeout_ = number(parameters, "feedback_timeout_s");
    startup_timeout_ = number(parameters, "startup_timeout_s");
    const double rate = number(parameters, "poll_rate_hz");
    max_control_period_ = number(parameters, "max_control_period_s");
    position_limit_tolerance_rad_ = parameters.count("position_limit_tolerance_rad") ?
      number(parameters, "position_limit_tolerance_rad") : 0.05;
    if (position_limit_tolerance_rad_ < 0) {
      throw std::invalid_argument("position_limit_tolerance_rad must be nonnegative");
    }
    const double watchdog_ms = number(parameters, "watchdog_ms");
    if (rate < 1 || rate > 100 || feedback_timeout_ < 2 / rate || startup_timeout_ <= 0 ||
      startup_timeout_ > 10 || max_control_period_ <= 0 ||
      max_control_period_ >= feedback_timeout_ || watchdog_ms < 20 || watchdog_ms > 1000 ||
      feedback_timeout_ * 1000 >= watchdog_ms)
    {
      throw std::invalid_argument("Invalid poll, feedback, startup, control-period or watchdog timing");
    }
    if (!read_only_ && !calibrated_) {
      throw std::invalid_argument("Motion requires calibration_confirmed=true");
    }
    poll_period_ = 1 / rate;
    watchdog_ticks_ = static_cast<uint32_t>(std::lround(watchdog_ms * 20));
    if (info_.joints.empty() || info_.joints.size() > 32) {
      throw std::invalid_argument("Expected 1..32 ros2_control joints");
    }
    std::map<std::string, std::size_t> joints;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      if (!joints.emplace(info_.joints[i].name, i).second) {
        throw std::invalid_argument("Duplicate joint name");
      }
    }
    std::set<uint8_t> motor_ids;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & joint = info_.joints[i];
      if (!has_interfaces(joint.state_interfaces)) {
        throw std::invalid_argument(joint.name + " requires position and velocity state interfaces");
      }
      const auto & p = joint.parameters;
      if (p.count("mimic")) {
        if (!joint.command_interfaces.empty() || !joints.count(p.at("mimic")) ||
          joints.at(p.at("mimic")) == i || p.count("motor_id"))
        {
          throw std::invalid_argument("Mimic must have a valid parent and no command interface/motor ID");
        }
        mimics_.push_back({i, joints.at(p.at("mimic")), number(p, "multiplier"), number(p, "offset")});
        continue;
      }
      if ((!read_only_ && (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != "position")) ||
        (read_only_ && !joint.command_interfaces.empty()))
      {
        throw std::invalid_argument("Command interfaces must match read_only mode");
      }
      Motor motor;
      motor.joint = i;
      motor.id = id(p, "motor_id");
      if (motor.id == host_ || !motor_ids.insert(motor.id).second) {
        throw std::invalid_argument("Duplicate motor ID or motor ID equals host ID");
      }
      motor.direction = number(p, "direction");
      motor.motor_zero = number(p, "motor_zero_rad");
      motor.joint_zero = number(p, "joint_zero");
      motor.scale = number(p, "motor_radians_per_joint_unit");
      motor.lower = number(p, "position_min");
      motor.upper = number(p, "position_max");
      motor.velocity = number(p, "max_velocity");
      motor.tracking_error = number(p, "max_tracking_error");
      motor.current = number(p, "current_limit_a");
      if ((motor.direction != 1 && motor.direction != -1) || motor.scale <= 0 ||
        motor.lower >= motor.upper || motor.velocity <= 0 || motor.tracking_error <= 0 ||
        motor.current < 0 || (!read_only_ && motor.current <= 0) ||
        !std::isfinite(motor.velocity * motor.scale) || motor.velocity * motor.scale > 20)
      {
        throw std::invalid_argument("Invalid calibration/limits for " + joint.name);
      }
      motors_.push_back(motor);
    }
    for (const auto & mimic : mimics_) {
      if (std::none_of(motors_.begin(), motors_.end(),
        [&](const Motor & motor) {return motor.joint == mimic.parent;}))
      {
        throw std::invalid_argument("Mimic chains/cycles are unsupported; parent must be a motor joint");
      }
    }
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    positions_.assign(info_.joints.size(), nan);
    velocities_.assign(info_.joints.size(), nan);
    commands_.assign(info_.joints.size(), nan);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Hardware initialization: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> RSSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    result.emplace_back(info_.joints[i].name, "position", &positions_[i]);
    result.emplace_back(info_.joints[i].name, "velocity", &velocities_[i]);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface> RSSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  if (!read_only_) {
    for (const auto & motor : motors_) {
      result.emplace_back(info_.joints[motor.joint].name, "position", &commands_[motor.joint]);
    }
  }
  return result;
}

void RSSystem::receive()
{
  can_frame frame{};
  // Bounded drain: traffic addressed to other hosts is filtered in the socket.
  for (std::size_t count = 0; count < 256 && bus_->receive(frame); ++count) {
    const auto reply = protocol::decode(frame, host_);
    if (!reply) {continue;}
    const auto found = std::find_if(motors_.begin(), motors_.end(),
      [&](const Motor & motor) {return motor.id == reply->motor_id;});
    if (found == motors_.end()) {continue;}
    auto & motor = *found;
    if (reply->type == 17) {
      if (reply->flags) {
        throw std::runtime_error("Motor " + std::to_string(motor.id) + " rejected parameter read");
      }
      if (pending_ && pending_->motor == motor.id && pending_->index == reply->index) {
        pending_->reply = reply;
      }
      if (reply->index == protocol::kPosition || reply->index == protocol::kVelocity) {
        const double value = reply->as_float();
        if (!std::isfinite(value)) {throw std::runtime_error("Non-finite motor feedback");}
        if (reply->index == protocol::kPosition) {
          motor.motor_position = value;
          const double raw_position = motor.to_joint(value);
          if (!std::isfinite(raw_position)) {throw std::runtime_error("Joint conversion overflow");}
          const double bounded = std::clamp(raw_position, motor.lower, motor.upper);
          const double excess_rad = std::abs(raw_position - bounded) * motor.scale;
          // Encoder angles arrive as float32. Allow one rounding unit at the
          // threshold so a wire value of 0.05f is not rejected as > 0.05 rad.
          const double rounding_rad = std::numeric_limits<float>::epsilon() *
            std::max({1.0, std::abs(value), std::abs(motor.motor_zero)});
          if (!std::isfinite(excess_rad) ||
            excess_rad > position_limit_tolerance_rad_ + rounding_rad)
          {
            throw std::runtime_error("Motor " + std::to_string(motor.id) +
              " position exceeds limit tolerance: raw_joint=" + std::to_string(raw_position) +
              ", excess_motor_rad=" + std::to_string(excess_rad) +
              ", tolerance_rad=" + std::to_string(position_limit_tolerance_rad_));
          }
          // Keep motor_position raw for diagnostics and tracking-error checks;
          // publish a bounded state so MoveIt and initial targets stay valid.
          positions_[motor.joint] = bounded;
          if (raw_position != bounded && !motor.position_clamped &&
            (motor.clamp_warning_at == Clock::time_point{} ||
            std::chrono::duration<double>(now() - motor.clamp_warning_at).count() >= 5.0))
          {
            RCLCPP_WARN(logger,
              "Motor %u: raw joint %.9f clamped to %.9f (within %.6f motor rad limit tolerance)",
              motor.id, raw_position, bounded, position_limit_tolerance_rad_);
            motor.clamp_warning_at = now();
          }
          motor.position_clamped = raw_position != bounded;
          motor.position_at = now();
        } else {
          velocities_[motor.joint] = motor.direction * value / motor.scale;
          if (!std::isfinite(velocities_[motor.joint])) {throw std::runtime_error("Joint velocity conversion overflow");}
          motor.velocity_at = now();
        }
      }
    } else if (reply->type == 2 || reply->type == 24) {
      if (reply->flags & 0x3F) {
        throw std::runtime_error("Motor " + std::to_string(motor.id) + " reports fault flags " +
          std::to_string(reply->flags & 0x3F));
      }
      motor.mode = reply->flags >> 6;
      motor.status_at = now();
      if (active_ && motor.owned && motor.mode != 2) {
        throw std::runtime_error("Motor left Run mode; refusing automatic re-enable");
      }
    } else if (reply->type == 21 && (reply->fault || reply->warning)) {
      throw std::runtime_error("Motor " + std::to_string(motor.id) + " fault=" +
        std::to_string(reply->fault) + " warning=" + std::to_string(reply->warning));
    }
  }
  update_mimics();
}

void RSSystem::update_mimics()
{
  for (const auto & mimic : mimics_) {
    positions_[mimic.joint] = mimic.factor * positions_[mimic.parent] + mimic.offset;
    velocities_[mimic.joint] = mimic.factor * velocities_[mimic.parent];
  }
}

void RSSystem::query_states()
{
  for (const auto & motor : motors_) {
    bus_->send(protocol::read_parameter(host_, motor.id, protocol::kPosition));
    bus_->send(protocol::read_parameter(host_, motor.id, protocol::kVelocity));
  }
  last_poll_ = now();
}

bool RSSystem::fresh() const
{
  return std::all_of(motors_.begin(), motors_.end(), [&](const Motor & motor) {
      return motor.position_at != Clock::time_point{} && motor.velocity_at != Clock::time_point{} &&
             std::chrono::duration<double>(now() - motor.position_at).count() < feedback_timeout_ &&
             std::chrono::duration<double>(now() - motor.velocity_at).count() < feedback_timeout_;
    });
}

void RSSystem::wait_for_states()
{
  const auto deadline = now() + std::chrono::duration<double>(startup_timeout_);
  query_states();
  while (now() < deadline) {
    receive();
    if (fresh()) {return;}
    if (std::chrono::duration<double>(now() - last_poll_).count() >= poll_period_) {query_states();}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("Timed out reading position/velocity from all configured motors");
}

void RSSystem::verify(
  const Motor & motor, const can_frame & command, uint16_t index, uint32_t expected)
{
  receive();
  pending_ = Pending{motor.id, index, std::nullopt};
  bus_->send(command);
  bus_->send(protocol::read_parameter(host_, motor.id, index));
  const auto deadline = now() + std::chrono::duration<double>(startup_timeout_);
  while (now() < deadline) {
    receive();
    if (pending_->reply) {
      const auto value = pending_->reply->as_u32();
      pending_.reset();
      if (value != expected) {
        throw std::runtime_error("Motor parameter readback mismatch for index " + std::to_string(index));
      }
      return;
    }
    if (std::chrono::duration<double>(now() - last_poll_).count() >= poll_period_) {query_states();}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  pending_.reset();
  throw std::runtime_error("Motor parameter readback timed out for index " + std::to_string(index));
}

RSSystem::CallbackReturn RSSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    close();
    faulted_ = false;
    for (auto & motor : motors_) {
      motor.position_at = {};
      motor.velocity_at = {};
      motor.status_at = {};
      motor.owned = false;
      motor.position_clamped = false;
      motor.clamp_warning_at = {};
    }
    bus_ = make_transport();
    bus_->open(interface_, host_);
    wait_for_states();
    RCLCPP_INFO(logger, "Received real encoder feedback from %zu motors on %s (%s)",
      motors_.size(), interface_.c_str(), read_only_ ? "read-only" : "motion not yet activated");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    fail(error.what());
    close();
    return CallbackReturn::ERROR;
  }
}

RSSystem::CallbackReturn RSSystem::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    if (!bus_ || faulted_) {throw std::runtime_error("Hardware is not configured");}
    wait_for_states();
    if (!read_only_) {
      for (auto & motor : motors_) {
        const double position = positions_[motor.joint];
        if (position < motor.lower || position > motor.upper) {
          throw std::runtime_error("Measured joint position outside configured limits; check zero/direction");
        }
        bus_->send(protocol::stop(host_, motor.id));
        verify(motor, protocol::write_u8(host_, motor.id, protocol::kRunMode, protocol::kCspMode),
          protocol::kRunMode, protocol::kCspMode);
        for (const auto & parameter : std::vector<std::pair<uint16_t, float>>{
          {protocol::kSpeedLimit, float(motor.velocity * motor.scale)},
          {protocol::kCurrentLimit, float(motor.current)}})
        {
          const auto frame = protocol::write_float(host_, motor.id, parameter.first, parameter.second);
          const uint32_t bits = uint32_t(frame.data[4]) | (uint32_t(frame.data[5]) << 8) |
            (uint32_t(frame.data[6]) << 16) | (uint32_t(frame.data[7]) << 24);
          verify(motor, frame, parameter.first, bits);
        }
        verify(motor, protocol::write_u32(host_, motor.id, protocol::kCanTimeout, watchdog_ticks_),
          protocol::kCanTimeout, watchdog_ticks_);
      }
      // Refresh after parameter handshakes; never use simulation initial_positions.
      wait_for_states();
      for (auto & motor : motors_) {
        if (positions_[motor.joint] < motor.lower || positions_[motor.joint] > motor.upper) {
          throw std::runtime_error("Joint moved outside its limits during activation");
        }
        motor.last_sent = positions_[motor.joint];
        commands_[motor.joint] = motor.last_sent;
        bus_->send(protocol::write_float(host_, motor.id, protocol::kPositionTarget,
          float(motor.to_motor(motor.last_sent))));
      }
      for (auto & motor : motors_) {
        const auto sent_at = now();
        motor.owned = true;  // Also stop this motor if its enable acknowledgment is lost.
        bus_->send(protocol::enable(host_, motor.id));
        bus_->send(protocol::write_float(host_, motor.id, protocol::kPositionTarget,
          float(motor.to_motor(motor.last_sent))));
        while (motor.status_at < sent_at || motor.mode != 2) {
          receive();
          if (std::chrono::duration<double>(now() - sent_at).count() >= startup_timeout_) {
            throw std::runtime_error("Motor enable did not acknowledge Run mode");
          }
          if (std::chrono::duration<double>(now() - last_poll_).count() >= poll_period_) {
            query_states();
            for (const auto & held : motors_) {
              if (held.owned) {
                bus_->send(protocol::write_float(host_, held.id, protocol::kPositionTarget,
                  float(held.to_motor(held.last_sent))));
              }
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
    }
    activated_at_ = now();
    first_write_ = true;
    active_ = true;
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    fail(error.what());
    return CallbackReturn::ERROR;
  }
}

hardware_interface::return_type RSSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_ || faulted_) {return hardware_interface::return_type::ERROR;}
  try {
    receive();
    if (std::chrono::duration<double>(now() - last_poll_).count() >= poll_period_) {
      query_states();
      receive();
    }
    if (!fresh()) {throw std::runtime_error("Motor position/velocity feedback is stale");}
    if (active_ && !read_only_) {
      for (const auto & motor : motors_) {
        if (positions_[motor.joint] < motor.lower || positions_[motor.joint] > motor.upper ||
          std::abs(motor.to_joint(motor.motor_position) - motor.last_sent) > motor.tracking_error)
        {
          throw std::runtime_error("Joint tracking error or position limit exceeded");
        }
      }
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception & error) {
    fail(error.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type RSSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_ || faulted_) {return hardware_interface::return_type::ERROR;}
  if (read_only_) {return hardware_interface::return_type::OK;}
  try {
    double dt = period.seconds();
    const double since_activation = std::chrono::duration<double>(now() - activated_at_).count();
    if (first_write_) {
      first_write_ = false;
      if (!(since_activation >= 0 && since_activation <= max_control_period_)) {
        throw std::runtime_error("First control write arrived too late after activation");
      }
    }
    // Depending on callback ordering, activation time can appear in either
    // of the first manager periods. Only while still inside the first regular
    // control deadline after activation, ignore that pre-activation time and
    // resend the held targets without advancing them (dt=0).
    const bool startup_period = std::isfinite(dt) && dt > max_control_period_ &&
      since_activation >= 0 && since_activation <= max_control_period_;
    if (startup_period) {dt = 0;}
    if ((!startup_period && !(dt > 0 && dt <= max_control_period_)) || !fresh()) {
      throw std::runtime_error("Control-loop period exceeded or motor feedback is stale: period_s=" +
        std::to_string(period.seconds()) + ", since_activation_s=" + std::to_string(since_activation));
    }
    // Validate the entire command set before sending anything to any motor.
    for (const auto & motor : motors_) {
      const double command = commands_[motor.joint];
      if (!std::isfinite(command) || command < motor.lower || command > motor.upper) {
        throw std::runtime_error("Invalid/non-finite/out-of-range joint command");
      }
    }
    for (auto & motor : motors_) {
      const double step = motor.velocity * dt;
      const double target = std::clamp(commands_[motor.joint], motor.last_sent - step, motor.last_sent + step);
      bus_->send(protocol::write_float(host_, motor.id, protocol::kPositionTarget, float(motor.to_motor(target))));
      motor.last_sent = target;
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception & error) {
    fail(error.what());
    return hardware_interface::return_type::ERROR;
  }
}

void RSSystem::stop_owned() noexcept
{
  active_ = false;
  if (!bus_) {return;}
  for (auto & motor : motors_) {
    if (!motor.owned) {continue;}
    try {bus_->send(protocol::stop(host_, motor.id));} catch (...) {
      RCLCPP_ERROR(logger, "Could not send stop to motor %u; firmware watchdog remains configured", motor.id);
    }
    motor.owned = false;
  }
}

void RSSystem::fail(const std::string & reason) noexcept
{
  faulted_ = true;
  RCLCPP_ERROR(logger, "Hardware fault latched: %s", reason.c_str());
  stop_owned();
}

void RSSystem::close() noexcept
{
  stop_owned();
  pending_.reset();
  if (bus_) {bus_->close();}
  bus_.reset();
}

RSSystem::CallbackReturn RSSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_owned();
  return CallbackReturn::SUCCESS;
}
RSSystem::CallbackReturn RSSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  close();
  return CallbackReturn::SUCCESS;
}
RSSystem::CallbackReturn RSSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  close();
  return CallbackReturn::SUCCESS;
}
RSSystem::CallbackReturn RSSystem::on_error(const rclcpp_lifecycle::State &)
{
  close();
  return CallbackReturn::SUCCESS;
}
}  // namespace rebot_arm_rs_hardware

PLUGINLIB_EXPORT_CLASS(rebot_arm_rs_hardware::RSSystem, hardware_interface::SystemInterface)
