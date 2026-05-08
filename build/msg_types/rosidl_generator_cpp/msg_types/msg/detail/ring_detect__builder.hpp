// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_types:msg/RingDetect.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/ring_detect.hpp"


#ifndef MSG_TYPES__MSG__DETAIL__RING_DETECT__BUILDER_HPP_
#define MSG_TYPES__MSG__DETAIL__RING_DETECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_types/msg/detail/ring_detect__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_types
{

namespace msg
{

namespace builder
{

class Init_RingDetect_color
{
public:
  explicit Init_RingDetect_color(::msg_types::msg::RingDetect & msg)
  : msg_(msg)
  {}
  ::msg_types::msg::RingDetect color(::msg_types::msg::RingDetect::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_types::msg::RingDetect msg_;
};

class Init_RingDetect_id
{
public:
  explicit Init_RingDetect_id(::msg_types::msg::RingDetect & msg)
  : msg_(msg)
  {}
  Init_RingDetect_color id(::msg_types::msg::RingDetect::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_RingDetect_color(msg_);
  }

private:
  ::msg_types::msg::RingDetect msg_;
};

class Init_RingDetect_y
{
public:
  explicit Init_RingDetect_y(::msg_types::msg::RingDetect & msg)
  : msg_(msg)
  {}
  Init_RingDetect_id y(::msg_types::msg::RingDetect::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RingDetect_id(msg_);
  }

private:
  ::msg_types::msg::RingDetect msg_;
};

class Init_RingDetect_x
{
public:
  Init_RingDetect_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RingDetect_y x(::msg_types::msg::RingDetect::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RingDetect_y(msg_);
  }

private:
  ::msg_types::msg::RingDetect msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_types::msg::RingDetect>()
{
  return msg_types::msg::builder::Init_RingDetect_x();
}

}  // namespace msg_types

#endif  // MSG_TYPES__MSG__DETAIL__RING_DETECT__BUILDER_HPP_
