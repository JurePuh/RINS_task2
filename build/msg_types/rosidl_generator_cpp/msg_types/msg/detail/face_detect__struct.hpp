// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_types:msg/FaceDetect.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/face_detect.hpp"


#ifndef MSG_TYPES__MSG__DETAIL__FACE_DETECT__STRUCT_HPP_
#define MSG_TYPES__MSG__DETAIL__FACE_DETECT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_types__msg__FaceDetect __attribute__((deprecated))
#else
# define DEPRECATED__msg_types__msg__FaceDetect __declspec(deprecated)
#endif

namespace msg_types
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FaceDetect_
{
  using Type = FaceDetect_<ContainerAllocator>;

  explicit FaceDetect_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->id = 0;
    }
  }

  explicit FaceDetect_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->id = 0;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _id_type =
    int8_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__id(
    const int8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_types::msg::FaceDetect_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_types::msg::FaceDetect_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_types::msg::FaceDetect_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_types::msg::FaceDetect_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_types::msg::FaceDetect_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_types::msg::FaceDetect_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_types::msg::FaceDetect_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_types::msg::FaceDetect_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_types::msg::FaceDetect_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_types::msg::FaceDetect_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_types__msg__FaceDetect
    std::shared_ptr<msg_types::msg::FaceDetect_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_types__msg__FaceDetect
    std::shared_ptr<msg_types::msg::FaceDetect_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FaceDetect_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const FaceDetect_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FaceDetect_

// alias to use template instance with default allocator
using FaceDetect =
  msg_types::msg::FaceDetect_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_types

#endif  // MSG_TYPES__MSG__DETAIL__FACE_DETECT__STRUCT_HPP_
