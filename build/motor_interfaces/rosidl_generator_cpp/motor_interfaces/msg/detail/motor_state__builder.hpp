// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_interfaces:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_interfaces/msg/motor_state.hpp"


#ifndef MOTOR_INTERFACES__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define MOTOR_INTERFACES__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_interfaces/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorState_temperature
{
public:
  explicit Init_MotorState_temperature(::motor_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::motor_interfaces::msg::MotorState temperature(::motor_interfaces::msg::MotorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_interfaces::msg::MotorState msg_;
};

class Init_MotorState_current
{
public:
  explicit Init_MotorState_current(::motor_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_temperature current(::motor_interfaces::msg::MotorState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_MotorState_temperature(msg_);
  }

private:
  ::motor_interfaces::msg::MotorState msg_;
};

class Init_MotorState_torque
{
public:
  explicit Init_MotorState_torque(::motor_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_current torque(::motor_interfaces::msg::MotorState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_MotorState_current(msg_);
  }

private:
  ::motor_interfaces::msg::MotorState msg_;
};

class Init_MotorState_velocity
{
public:
  explicit Init_MotorState_velocity(::motor_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_torque velocity(::motor_interfaces::msg::MotorState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorState_torque(msg_);
  }

private:
  ::motor_interfaces::msg::MotorState msg_;
};

class Init_MotorState_abs_position
{
public:
  explicit Init_MotorState_abs_position(::motor_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_velocity abs_position(::motor_interfaces::msg::MotorState::_abs_position_type arg)
  {
    msg_.abs_position = std::move(arg);
    return Init_MotorState_velocity(msg_);
  }

private:
  ::motor_interfaces::msg::MotorState msg_;
};

class Init_MotorState_position
{
public:
  explicit Init_MotorState_position(::motor_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_abs_position position(::motor_interfaces::msg::MotorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorState_abs_position(msg_);
  }

private:
  ::motor_interfaces::msg::MotorState msg_;
};

class Init_MotorState_name
{
public:
  Init_MotorState_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_position name(::motor_interfaces::msg::MotorState::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_MotorState_position(msg_);
  }

private:
  ::motor_interfaces::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_interfaces::msg::MotorState>()
{
  return motor_interfaces::msg::builder::Init_MotorState_name();
}

}  // namespace motor_interfaces

#endif  // MOTOR_INTERFACES__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
