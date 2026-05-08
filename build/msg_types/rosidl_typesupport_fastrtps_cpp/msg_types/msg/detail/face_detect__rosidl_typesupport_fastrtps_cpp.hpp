// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from msg_types:msg/FaceDetect.idl
// generated code does not contain a copyright notice

#ifndef MSG_TYPES__MSG__DETAIL__FACE_DETECT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MSG_TYPES__MSG__DETAIL__FACE_DETECT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "msg_types/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "msg_types/msg/detail/face_detect__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace msg_types
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_types
cdr_serialize(
  const msg_types::msg::FaceDetect & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_types
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  msg_types::msg::FaceDetect & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_types
get_serialized_size(
  const msg_types::msg::FaceDetect & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_types
max_serialized_size_FaceDetect(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_types
cdr_serialize_key(
  const msg_types::msg::FaceDetect & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_types
get_serialized_size_key(
  const msg_types::msg::FaceDetect & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_types
max_serialized_size_key_FaceDetect(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace msg_types

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_types
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, msg_types, msg, FaceDetect)();

#ifdef __cplusplus
}
#endif

#endif  // MSG_TYPES__MSG__DETAIL__FACE_DETECT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
