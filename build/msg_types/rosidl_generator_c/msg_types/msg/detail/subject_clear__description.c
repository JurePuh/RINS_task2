// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from msg_types:msg/SubjectClear.idl
// generated code does not contain a copyright notice

#include "msg_types/msg/detail/subject_clear__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_msg_types
const rosidl_type_hash_t *
msg_types__msg__SubjectClear__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe5, 0xc7, 0x86, 0xe2, 0xcf, 0xe2, 0xf6, 0x0f,
      0xb9, 0x81, 0xd0, 0x08, 0xe6, 0xf5, 0x2c, 0xaa,
      0x99, 0x83, 0xca, 0x6c, 0xa0, 0x92, 0xbc, 0xef,
      0x53, 0x21, 0xb8, 0x0f, 0xea, 0x5c, 0x32, 0x50,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char msg_types__msg__SubjectClear__TYPE_NAME[] = "msg_types/msg/SubjectClear";

// Define type names, field names, and default values
static char msg_types__msg__SubjectClear__FIELD_NAME__id[] = "id";

static rosidl_runtime_c__type_description__Field msg_types__msg__SubjectClear__FIELDS[] = {
  {
    {msg_types__msg__SubjectClear__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
msg_types__msg__SubjectClear__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {msg_types__msg__SubjectClear__TYPE_NAME, 26, 26},
      {msg_types__msg__SubjectClear__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int8 id";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
msg_types__msg__SubjectClear__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {msg_types__msg__SubjectClear__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 7, 7},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
msg_types__msg__SubjectClear__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *msg_types__msg__SubjectClear__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
