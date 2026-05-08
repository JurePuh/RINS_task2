// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from msg_types:msg/RingDetect.idl
// generated code does not contain a copyright notice

#include "msg_types/msg/detail/ring_detect__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_msg_types
const rosidl_type_hash_t *
msg_types__msg__RingDetect__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x62, 0xfd, 0x4b, 0xc8, 0xd6, 0x52, 0x07, 0x12,
      0xe6, 0x81, 0x05, 0xaa, 0xd9, 0x16, 0x36, 0x73,
      0x85, 0xdc, 0x7b, 0xcb, 0x9d, 0xb4, 0xab, 0xc5,
      0xc1, 0x55, 0x36, 0x4a, 0xcb, 0xd8, 0x9a, 0xe1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char msg_types__msg__RingDetect__TYPE_NAME[] = "msg_types/msg/RingDetect";

// Define type names, field names, and default values
static char msg_types__msg__RingDetect__FIELD_NAME__x[] = "x";
static char msg_types__msg__RingDetect__FIELD_NAME__y[] = "y";
static char msg_types__msg__RingDetect__FIELD_NAME__id[] = "id";
static char msg_types__msg__RingDetect__FIELD_NAME__color[] = "color";

static rosidl_runtime_c__type_description__Field msg_types__msg__RingDetect__FIELDS[] = {
  {
    {msg_types__msg__RingDetect__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {msg_types__msg__RingDetect__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {msg_types__msg__RingDetect__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {msg_types__msg__RingDetect__FIELD_NAME__color, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
msg_types__msg__RingDetect__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {msg_types__msg__RingDetect__TYPE_NAME, 24, 24},
      {msg_types__msg__RingDetect__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 x\n"
  "float32 y\n"
  "int8 id\n"
  "string color";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
msg_types__msg__RingDetect__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {msg_types__msg__RingDetect__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 40, 40},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
msg_types__msg__RingDetect__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *msg_types__msg__RingDetect__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
