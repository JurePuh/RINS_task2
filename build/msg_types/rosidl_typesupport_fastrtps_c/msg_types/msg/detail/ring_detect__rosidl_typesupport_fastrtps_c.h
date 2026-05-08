// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from msg_types:msg/RingDetect.idl
// generated code does not contain a copyright notice
#ifndef MSG_TYPES__MSG__DETAIL__RING_DETECT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MSG_TYPES__MSG__DETAIL__RING_DETECT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "msg_types/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "msg_types/msg/detail/ring_detect__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_types
bool cdr_serialize_msg_types__msg__RingDetect(
  const msg_types__msg__RingDetect * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_types
bool cdr_deserialize_msg_types__msg__RingDetect(
  eprosima::fastcdr::Cdr &,
  msg_types__msg__RingDetect * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_types
size_t get_serialized_size_msg_types__msg__RingDetect(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_types
size_t max_serialized_size_msg_types__msg__RingDetect(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_types
bool cdr_serialize_key_msg_types__msg__RingDetect(
  const msg_types__msg__RingDetect * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_types
size_t get_serialized_size_key_msg_types__msg__RingDetect(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_types
size_t max_serialized_size_key_msg_types__msg__RingDetect(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_types
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, msg_types, msg, RingDetect)();

#ifdef __cplusplus
}
#endif

#endif  // MSG_TYPES__MSG__DETAIL__RING_DETECT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
