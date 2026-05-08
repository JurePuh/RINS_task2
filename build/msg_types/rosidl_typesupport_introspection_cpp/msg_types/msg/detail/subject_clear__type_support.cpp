// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from msg_types:msg/SubjectClear.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "msg_types/msg/detail/subject_clear__functions.h"
#include "msg_types/msg/detail/subject_clear__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace msg_types
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SubjectClear_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) msg_types::msg::SubjectClear(_init);
}

void SubjectClear_fini_function(void * message_memory)
{
  auto typed_message = static_cast<msg_types::msg::SubjectClear *>(message_memory);
  typed_message->~SubjectClear();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SubjectClear_message_member_array[1] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_types::msg::SubjectClear, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SubjectClear_message_members = {
  "msg_types::msg",  // message namespace
  "SubjectClear",  // message name
  1,  // number of fields
  sizeof(msg_types::msg::SubjectClear),
  false,  // has_any_key_member_
  SubjectClear_message_member_array,  // message members
  SubjectClear_init_function,  // function to initialize message memory (memory has to be allocated)
  SubjectClear_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SubjectClear_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SubjectClear_message_members,
  get_message_typesupport_handle_function,
  &msg_types__msg__SubjectClear__get_type_hash,
  &msg_types__msg__SubjectClear__get_type_description,
  &msg_types__msg__SubjectClear__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace msg_types


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<msg_types::msg::SubjectClear>()
{
  return &::msg_types::msg::rosidl_typesupport_introspection_cpp::SubjectClear_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, msg_types, msg, SubjectClear)() {
  return &::msg_types::msg::rosidl_typesupport_introspection_cpp::SubjectClear_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
