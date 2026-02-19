// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_interfaces:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_interfaces/msg/motor_state.h"


#ifndef MOTOR_INTERFACES__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
#define MOTOR_INTERFACES__MSG__DETAIL__MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotorState in the package motor_interfaces.
/**
  * This message contains data used to describe the current state of CubeMars v2 motors
 */
typedef struct motor_interfaces__msg__MotorState
{
  /// The state of each motor is defined by:
  ///   - position of the motor (rad)
  ///   - absolute position of the motor (rad)
  ///   - velocity of the motor (rad/s)
  ///   - current applied (A)
  ///   - torque applied (nm)
  ///   - temperature (C)
  ///
  /// Each motor is uniquely identified by its name
  rosidl_runtime_c__String name;
  float position;
  float abs_position;
  float velocity;
  float torque;
  float current;
  int32_t temperature;
} motor_interfaces__msg__MotorState;

// Struct for a sequence of motor_interfaces__msg__MotorState.
typedef struct motor_interfaces__msg__MotorState__Sequence
{
  motor_interfaces__msg__MotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_interfaces__msg__MotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_INTERFACES__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
