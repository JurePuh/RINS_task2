// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_types:msg/SubjectClear.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/subject_clear.h"


#ifndef MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__STRUCT_H_
#define MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/SubjectClear in the package msg_types.
typedef struct msg_types__msg__SubjectClear
{
  int8_t id;
} msg_types__msg__SubjectClear;

// Struct for a sequence of msg_types__msg__SubjectClear.
typedef struct msg_types__msg__SubjectClear__Sequence
{
  msg_types__msg__SubjectClear * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_types__msg__SubjectClear__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__STRUCT_H_
