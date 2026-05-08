// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_types:msg/RingDetect.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/ring_detect.h"


#ifndef MSG_TYPES__MSG__DETAIL__RING_DETECT__STRUCT_H_
#define MSG_TYPES__MSG__DETAIL__RING_DETECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RingDetect in the package msg_types.
typedef struct msg_types__msg__RingDetect
{
  float x;
  float y;
  int8_t id;
  rosidl_runtime_c__String color;
} msg_types__msg__RingDetect;

// Struct for a sequence of msg_types__msg__RingDetect.
typedef struct msg_types__msg__RingDetect__Sequence
{
  msg_types__msg__RingDetect * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_types__msg__RingDetect__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_TYPES__MSG__DETAIL__RING_DETECT__STRUCT_H_
