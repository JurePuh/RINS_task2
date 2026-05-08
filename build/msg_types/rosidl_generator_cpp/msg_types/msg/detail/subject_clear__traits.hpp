// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_types:msg/SubjectClear.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/subject_clear.hpp"


#ifndef MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__TRAITS_HPP_
#define MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_types/msg/detail/subject_clear__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_types
{

namespace msg
{

inline void to_flow_style_yaml(
  const SubjectClear & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SubjectClear & msg,
  std::ostream & out, size_t indentation = 0)
{
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

inline std::string to_yaml(const SubjectClear & msg, bool use_flow_style = false)
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
  const msg_types::msg::SubjectClear & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_types::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_types::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_types::msg::SubjectClear & msg)
{
  return msg_types::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_types::msg::SubjectClear>()
{
  return "msg_types::msg::SubjectClear";
}

template<>
inline const char * name<msg_types::msg::SubjectClear>()
{
  return "msg_types/msg/SubjectClear";
}

template<>
struct has_fixed_size<msg_types::msg::SubjectClear>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_types::msg::SubjectClear>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_types::msg::SubjectClear>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__TRAITS_HPP_
