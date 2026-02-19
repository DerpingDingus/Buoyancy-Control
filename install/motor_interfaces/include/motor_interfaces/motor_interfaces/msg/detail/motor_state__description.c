// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from motor_interfaces:msg/MotorState.idl
// generated code does not contain a copyright notice

#include "motor_interfaces/msg/detail/motor_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_motor_interfaces
const rosidl_type_hash_t *
motor_interfaces__msg__MotorState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf9, 0x66, 0x54, 0xd0, 0x35, 0x3c, 0xdf, 0x42,
      0xe3, 0x18, 0xf8, 0x70, 0x3c, 0x86, 0x52, 0x3d,
      0x01, 0xe5, 0xb5, 0x24, 0x7e, 0x40, 0x3a, 0x4e,
      0x5d, 0x50, 0x6e, 0x9d, 0x11, 0x89, 0xa8, 0xa7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char motor_interfaces__msg__MotorState__TYPE_NAME[] = "motor_interfaces/msg/MotorState";

// Define type names, field names, and default values
static char motor_interfaces__msg__MotorState__FIELD_NAME__name[] = "name";
static char motor_interfaces__msg__MotorState__FIELD_NAME__position[] = "position";
static char motor_interfaces__msg__MotorState__FIELD_NAME__abs_position[] = "abs_position";
static char motor_interfaces__msg__MotorState__FIELD_NAME__velocity[] = "velocity";
static char motor_interfaces__msg__MotorState__FIELD_NAME__torque[] = "torque";
static char motor_interfaces__msg__MotorState__FIELD_NAME__current[] = "current";
static char motor_interfaces__msg__MotorState__FIELD_NAME__temperature[] = "temperature";

static rosidl_runtime_c__type_description__Field motor_interfaces__msg__MotorState__FIELDS[] = {
  {
    {motor_interfaces__msg__MotorState__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_interfaces__msg__MotorState__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_interfaces__msg__MotorState__FIELD_NAME__abs_position, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_interfaces__msg__MotorState__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_interfaces__msg__MotorState__FIELD_NAME__torque, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_interfaces__msg__MotorState__FIELD_NAME__current, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_interfaces__msg__MotorState__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
motor_interfaces__msg__MotorState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {motor_interfaces__msg__MotorState__TYPE_NAME, 31, 31},
      {motor_interfaces__msg__MotorState__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message contains data used to describe the current state of CubeMars v2 motors\n"
  "\n"
  "# The state of each motor is defined by:\n"
  "#   - position of the motor (rad)\n"
  "#   - absolute position of the motor (rad)\n"
  "#   - velocity of the motor (rad/s)\n"
  "#   - current applied (A)\n"
  "#   - torque applied (nm)\n"
  "#   - temperature (C)\n"
  "#\n"
  "# Each motor is uniquely identified by its name \n"
  "\n"
  "string name\n"
  "float32 position\n"
  "float32 abs_position \n"
  "float32 velocity\n"
  "float32 torque\n"
  "float32 current\n"
  "int32 temperature\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
motor_interfaces__msg__MotorState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {motor_interfaces__msg__MotorState__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 485, 485},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
motor_interfaces__msg__MotorState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *motor_interfaces__msg__MotorState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
