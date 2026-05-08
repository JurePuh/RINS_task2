// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_types:msg/RingDetect.idl
// generated code does not contain a copyright notice
#include "msg_types/msg/detail/ring_detect__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `color`
#include "rosidl_runtime_c/string_functions.h"

bool
msg_types__msg__RingDetect__init(msg_types__msg__RingDetect * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // id
  // color
  if (!rosidl_runtime_c__String__init(&msg->color)) {
    msg_types__msg__RingDetect__fini(msg);
    return false;
  }
  return true;
}

void
msg_types__msg__RingDetect__fini(msg_types__msg__RingDetect * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // id
  // color
  rosidl_runtime_c__String__fini(&msg->color);
}

bool
msg_types__msg__RingDetect__are_equal(const msg_types__msg__RingDetect * lhs, const msg_types__msg__RingDetect * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  return true;
}

bool
msg_types__msg__RingDetect__copy(
  const msg_types__msg__RingDetect * input,
  msg_types__msg__RingDetect * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // id
  output->id = input->id;
  // color
  if (!rosidl_runtime_c__String__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  return true;
}

msg_types__msg__RingDetect *
msg_types__msg__RingDetect__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_types__msg__RingDetect * msg = (msg_types__msg__RingDetect *)allocator.allocate(sizeof(msg_types__msg__RingDetect), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_types__msg__RingDetect));
  bool success = msg_types__msg__RingDetect__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msg_types__msg__RingDetect__destroy(msg_types__msg__RingDetect * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msg_types__msg__RingDetect__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msg_types__msg__RingDetect__Sequence__init(msg_types__msg__RingDetect__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_types__msg__RingDetect * data = NULL;

  if (size) {
    data = (msg_types__msg__RingDetect *)allocator.zero_allocate(size, sizeof(msg_types__msg__RingDetect), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_types__msg__RingDetect__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_types__msg__RingDetect__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
msg_types__msg__RingDetect__Sequence__fini(msg_types__msg__RingDetect__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      msg_types__msg__RingDetect__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

msg_types__msg__RingDetect__Sequence *
msg_types__msg__RingDetect__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_types__msg__RingDetect__Sequence * array = (msg_types__msg__RingDetect__Sequence *)allocator.allocate(sizeof(msg_types__msg__RingDetect__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msg_types__msg__RingDetect__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msg_types__msg__RingDetect__Sequence__destroy(msg_types__msg__RingDetect__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msg_types__msg__RingDetect__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msg_types__msg__RingDetect__Sequence__are_equal(const msg_types__msg__RingDetect__Sequence * lhs, const msg_types__msg__RingDetect__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msg_types__msg__RingDetect__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msg_types__msg__RingDetect__Sequence__copy(
  const msg_types__msg__RingDetect__Sequence * input,
  msg_types__msg__RingDetect__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msg_types__msg__RingDetect);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msg_types__msg__RingDetect * data =
      (msg_types__msg__RingDetect *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msg_types__msg__RingDetect__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msg_types__msg__RingDetect__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msg_types__msg__RingDetect__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
