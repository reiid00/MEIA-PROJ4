// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/GpioConfig.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__GPIO_CONFIG__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__GPIO_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'INPUT'.
enum
{
  px4_msgs__msg__GpioConfig__INPUT = 0ul
};

/// Constant 'OUTPUT'.
enum
{
  px4_msgs__msg__GpioConfig__OUTPUT = 1ul
};

/// Constant 'PULLUP'.
enum
{
  px4_msgs__msg__GpioConfig__PULLUP = 16ul
};

/// Constant 'PULLDOWN'.
enum
{
  px4_msgs__msg__GpioConfig__PULLDOWN = 32ul
};

/// Constant 'OPENDRAIN'.
enum
{
  px4_msgs__msg__GpioConfig__OPENDRAIN = 256ul
};

/// Constant 'INPUT_FLOATING'.
enum
{
  px4_msgs__msg__GpioConfig__INPUT_FLOATING = 0ul
};

/// Constant 'INPUT_PULLUP'.
enum
{
  px4_msgs__msg__GpioConfig__INPUT_PULLUP = 16ul
};

/// Constant 'INPUT_PULLDOWN'.
enum
{
  px4_msgs__msg__GpioConfig__INPUT_PULLDOWN = 32ul
};

/// Constant 'OUTPUT_PUSHPULL'.
enum
{
  px4_msgs__msg__GpioConfig__OUTPUT_PUSHPULL = 0ul
};

/// Constant 'OUTPUT_OPENDRAIN'.
enum
{
  px4_msgs__msg__GpioConfig__OUTPUT_OPENDRAIN = 256ul
};

/// Constant 'OUTPUT_OPENDRAIN_PULLUP'.
enum
{
  px4_msgs__msg__GpioConfig__OUTPUT_OPENDRAIN_PULLUP = 272ul
};

// Struct defined in msg/GpioConfig in the package px4_msgs.
typedef struct px4_msgs__msg__GpioConfig
{
  uint64_t timestamp;
  uint32_t device_id;
  uint32_t mask;
  uint32_t state;
  uint32_t config;
} px4_msgs__msg__GpioConfig;

// Struct for a sequence of px4_msgs__msg__GpioConfig.
typedef struct px4_msgs__msg__GpioConfig__Sequence
{
  px4_msgs__msg__GpioConfig * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__GpioConfig__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__GPIO_CONFIG__STRUCT_H_
