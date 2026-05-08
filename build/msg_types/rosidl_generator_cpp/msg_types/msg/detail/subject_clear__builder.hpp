// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_types:msg/SubjectClear.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/subject_clear.hpp"


#ifndef MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__BUILDER_HPP_
#define MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_types/msg/detail/subject_clear__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_types
{

namespace msg
{

namespace builder
{

class Init_SubjectClear_id
{
public:
  Init_SubjectClear_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::msg_types::msg::SubjectClear id(::msg_types::msg::SubjectClear::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_types::msg::SubjectClear msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_types::msg::SubjectClear>()
{
  return msg_types::msg::builder::Init_SubjectClear_id();
}

}  // namespace msg_types

#endif  // MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__BUILDER_HPP_
