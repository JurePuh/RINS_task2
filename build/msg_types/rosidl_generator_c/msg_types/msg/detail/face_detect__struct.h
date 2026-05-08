// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_types:msg/FaceDetect.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/face_detect.h"


#ifndef MSG_TYPES__MSG__DETAIL__FACE_DETECT__STRUCT_H_
#define MSG_TYPES__MSG__DETAIL__FACE_DETECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/FaceDetect in the package msg_types.
typedef struct msg_types__msg__FaceDetect
{
  float x;
  float y;
  int8_t id;
} msg_types__msg__FaceDetect;

// Struct for a sequence of msg_types__msg__FaceDetect.
typedef struct msg_types__msg__FaceDetect__Sequence
{
  msg_types__msg__FaceDetect * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_types__msg__FaceDetect__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_TYPES__MSG__DETAIL__FACE_DETECT__STRUCT_H_
