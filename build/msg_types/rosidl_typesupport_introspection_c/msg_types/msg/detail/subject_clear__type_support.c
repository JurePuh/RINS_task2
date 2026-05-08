// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from msg_types:msg/SubjectClear.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "msg_types/msg/detail/subject_clear__rosidl_typesupport_introspection_c.h"
#include "msg_types/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "msg_types/msg/detail/subject_clear__functions.h"
#include "msg_types/msg/detail/subject_clear__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  msg_types__msg__SubjectClear__init(message_memory);
}

void msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_fini_function(void * message_memory)
{
  msg_types__msg__SubjectClear__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_message_member_array[1] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_types__msg__SubjectClear, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_message_members = {
  "msg_types__msg",  // message namespace
  "SubjectClear",  // message name
  1,  // number of fields
  sizeof(msg_types__msg__SubjectClear),
  false,  // has_any_key_member_
  msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_message_member_array,  // message members
  msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_init_function,  // function to initialize message memory (memory has to be allocated)
  msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_message_type_support_handle = {
  0,
  &msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_message_members,
  get_message_typesupport_handle_function,
  &msg_types__msg__SubjectClear__get_type_hash,
  &msg_types__msg__SubjectClear__get_type_description,
  &msg_types__msg__SubjectClear__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_msg_types
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msg_types, msg, SubjectClear)() {
  if (!msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_message_type_support_handle.typesupport_identifier) {
    msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &msg_types__msg__SubjectClear__rosidl_typesupport_introspection_c__SubjectClear_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
