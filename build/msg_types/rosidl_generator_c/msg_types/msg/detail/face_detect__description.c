// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from msg_types:msg/FaceDetect.idl
// generated code does not contain a copyright notice

#include "msg_types/msg/detail/face_detect__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_msg_types
const rosidl_type_hash_t *
msg_types__msg__FaceDetect__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x66, 0x94, 0x1c, 0x7c, 0x42, 0x1e, 0x98, 0x45,
      0x4e, 0x27, 0x02, 0x88, 0xf8, 0xa4, 0x5a, 0xea,
      0x87, 0xf1, 0x14, 0x2c, 0x23, 0x69, 0xd7, 0xae,
      0x92, 0xce, 0x0b, 0x09, 0x88, 0x2d, 0x15, 0x92,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char msg_types__msg__FaceDetect__TYPE_NAME[] = "msg_types/msg/FaceDetect";

// Define type names, field names, and default values
static char msg_types__msg__FaceDetect__FIELD_NAME__x[] = "x";
static char msg_types__msg__FaceDetect__FIELD_NAME__y[] = "y";
static char msg_types__msg__FaceDetect__FIELD_NAME__id[] = "id";

static rosidl_runtime_c__type_description__Field msg_types__msg__FaceDetect__FIELDS[] = {
  {
    {msg_types__msg__FaceDetect__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {msg_types__msg__FaceDetect__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {msg_types__msg__FaceDetect__FIELD_NAME__id, 2, 2},
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
msg_types__msg__FaceDetect__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {msg_types__msg__FaceDetect__TYPE_NAME, 24, 24},
      {msg_types__msg__FaceDetect__FIELDS, 3, 3},
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
  "int8 id";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
msg_types__msg__FaceDetect__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {msg_types__msg__FaceDetect__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 27, 27},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
msg_types__msg__FaceDetect__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *msg_types__msg__FaceDetect__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
