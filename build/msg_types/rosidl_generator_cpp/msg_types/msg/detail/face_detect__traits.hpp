// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_types:msg/FaceDetect.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/face_detect.hpp"


#ifndef MSG_TYPES__MSG__DETAIL__FACE_DETECT__TRAITS_HPP_
#define MSG_TYPES__MSG__DETAIL__FACE_DETECT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_types/msg/detail/face_detect__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_types
{

namespace msg
{

inline void to_flow_style_yaml(
  const FaceDetect & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FaceDetect & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FaceDetect & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace msg_types

namespace rosidl_generator_traits
{

[[deprecated("use msg_types::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msg_types::msg::FaceDetect & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_types::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_types::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_types::msg::FaceDetect & msg)
{
  return msg_types::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_types::msg::FaceDetect>()
{
  return "msg_types::msg::FaceDetect";
}

template<>
inline const char * name<msg_types::msg::FaceDetect>()
{
  return "msg_types/msg/FaceDetect";
}

template<>
struct has_fixed_size<msg_types::msg::FaceDetect>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_types::msg::FaceDetect>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_types::msg::FaceDetect>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_TYPES__MSG__DETAIL__FACE_DETECT__TRAITS_HPP_
