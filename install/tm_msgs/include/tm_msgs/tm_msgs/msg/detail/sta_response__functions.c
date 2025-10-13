// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tm_msgs:msg/StaResponse.idl
// generated code does not contain a copyright notice
#include "tm_msgs/msg/detail/sta_response__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `subcmd`
// Member `subdata`
#include "rosidl_runtime_c/string_functions.h"

bool
tm_msgs__msg__StaResponse__init(tm_msgs__msg__StaResponse * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tm_msgs__msg__StaResponse__fini(msg);
    return false;
  }
  // subcmd
  if (!rosidl_runtime_c__String__init(&msg->subcmd)) {
    tm_msgs__msg__StaResponse__fini(msg);
    return false;
  }
  // subdata
  if (!rosidl_runtime_c__String__init(&msg->subdata)) {
    tm_msgs__msg__StaResponse__fini(msg);
    return false;
  }
  return true;
}

void
tm_msgs__msg__StaResponse__fini(tm_msgs__msg__StaResponse * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // subcmd
  rosidl_runtime_c__String__fini(&msg->subcmd);
  // subdata
  rosidl_runtime_c__String__fini(&msg->subdata);
}

bool
tm_msgs__msg__StaResponse__are_equal(const tm_msgs__msg__StaResponse * lhs, const tm_msgs__msg__StaResponse * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // subcmd
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->subcmd), &(rhs->subcmd)))
  {
    return false;
  }
  // subdata
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->subdata), &(rhs->subdata)))
  {
    return false;
  }
  return true;
}

bool
tm_msgs__msg__StaResponse__copy(
  const tm_msgs__msg__StaResponse * input,
  tm_msgs__msg__StaResponse * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // subcmd
  if (!rosidl_runtime_c__String__copy(
      &(input->subcmd), &(output->subcmd)))
  {
    return false;
  }
  // subdata
  if (!rosidl_runtime_c__String__copy(
      &(input->subdata), &(output->subdata)))
  {
    return false;
  }
  return true;
}

tm_msgs__msg__StaResponse *
tm_msgs__msg__StaResponse__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__StaResponse * msg = (tm_msgs__msg__StaResponse *)allocator.allocate(sizeof(tm_msgs__msg__StaResponse), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_msgs__msg__StaResponse));
  bool success = tm_msgs__msg__StaResponse__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_msgs__msg__StaResponse__destroy(tm_msgs__msg__StaResponse * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_msgs__msg__StaResponse__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_msgs__msg__StaResponse__Sequence__init(tm_msgs__msg__StaResponse__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__StaResponse * data = NULL;

  if (size) {
    data = (tm_msgs__msg__StaResponse *)allocator.zero_allocate(size, sizeof(tm_msgs__msg__StaResponse), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_msgs__msg__StaResponse__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_msgs__msg__StaResponse__fini(&data[i - 1]);
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
tm_msgs__msg__StaResponse__Sequence__fini(tm_msgs__msg__StaResponse__Sequence * array)
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
      tm_msgs__msg__StaResponse__fini(&array->data[i]);
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

tm_msgs__msg__StaResponse__Sequence *
tm_msgs__msg__StaResponse__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__StaResponse__Sequence * array = (tm_msgs__msg__StaResponse__Sequence *)allocator.allocate(sizeof(tm_msgs__msg__StaResponse__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_msgs__msg__StaResponse__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_msgs__msg__StaResponse__Sequence__destroy(tm_msgs__msg__StaResponse__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_msgs__msg__StaResponse__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_msgs__msg__StaResponse__Sequence__are_equal(const tm_msgs__msg__StaResponse__Sequence * lhs, const tm_msgs__msg__StaResponse__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_msgs__msg__StaResponse__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_msgs__msg__StaResponse__Sequence__copy(
  const tm_msgs__msg__StaResponse__Sequence * input,
  tm_msgs__msg__StaResponse__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_msgs__msg__StaResponse);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_msgs__msg__StaResponse * data =
      (tm_msgs__msg__StaResponse *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_msgs__msg__StaResponse__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_msgs__msg__StaResponse__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_msgs__msg__StaResponse__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
