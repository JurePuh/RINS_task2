// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_types:msg/SubjectClear.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "msg_types/msg/subject_clear.hpp"


#ifndef MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__STRUCT_HPP_
#define MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_types__msg__SubjectClear __attribute__((deprecated))
#else
# define DEPRECATED__msg_types__msg__SubjectClear __declspec(deprecated)
#endif

namespace msg_types
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SubjectClear_
{
  using Type = SubjectClear_<ContainerAllocator>;

  explicit SubjectClear_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
    }
  }

  explicit SubjectClear_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
    }
  }

  // field types and members
  using _id_type =
    int8_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__id(
    const int8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_types::msg::SubjectClear_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_types::msg::SubjectClear_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_types::msg::SubjectClear_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_types::msg::SubjectClear_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_types::msg::SubjectClear_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_types::msg::SubjectClear_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_types::msg::SubjectClear_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_types::msg::SubjectClear_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_types::msg::SubjectClear_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_types::msg::SubjectClear_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_types__msg__SubjectClear
    std::shared_ptr<msg_types::msg::SubjectClear_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_types__msg__SubjectClear
    std::shared_ptr<msg_types::msg::SubjectClear_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SubjectClear_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const SubjectClear_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SubjectClear_

// alias to use template instance with default allocator
using SubjectClear =
  msg_types::msg::SubjectClear_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_types

#endif  // MSG_TYPES__MSG__DETAIL__SUBJECT_CLEAR__STRUCT_HPP_
