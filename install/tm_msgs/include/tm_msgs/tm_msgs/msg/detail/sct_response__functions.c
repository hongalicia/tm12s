// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tm_msgs:msg/SctResponse.idl
// generated code does not contain a copyright notice
#include "tm_msgs/msg/detail/sct_response__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `id`
// Member `script`
#include "rosidl_runtime_c/string_functions.h"

bool
tm_msgs__msg__SctResponse__init(tm_msgs__msg__SctResponse * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tm_msgs__msg__SctResponse__fini(msg);
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__init(&msg->id)) {
    tm_msgs__msg__SctResponse__fini(msg);
    return false;
  }
  // script
  if (!rosidl_runtime_c__String__init(&msg->script)) {
    tm_msgs__msg__SctResponse__fini(msg);
    return false;
  }
  return true;
}

void
tm_msgs__msg__SctResponse__fini(tm_msgs__msg__SctResponse * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  rosidl_runtime_c__String__fini(&msg->id);
  // script
  rosidl_runtime_c__String__fini(&msg->script);
}

bool
tm_msgs__msg__SctResponse__are_equal(const tm_msgs__msg__SctResponse * lhs, const tm_msgs__msg__SctResponse * rhs)
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
  // id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  // script
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->script), &(rhs->script)))
  {
    return false;
  }
  return true;
}

bool
tm_msgs__msg__SctResponse__copy(
  const tm_msgs__msg__SctResponse * input,
  tm_msgs__msg__SctResponse * output)
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
  // id
  if (!rosidl_runtime_c__String__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  // script
  if (!rosidl_runtime_c__String__copy(
      &(input->script), &(output->script)))
  {
    return false;
  }
  return true;
}

tm_msgs__msg__SctResponse *
tm_msgs__msg__SctResponse__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__SctResponse * msg = (tm_msgs__msg__SctResponse *)allocator.allocate(sizeof(tm_msgs__msg__SctResponse), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_msgs__msg__SctResponse));
  bool success = tm_msgs__msg__SctResponse__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_msgs__msg__SctResponse__destroy(tm_msgs__msg__SctResponse * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_msgs__msg__SctResponse__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_msgs__msg__SctResponse__Sequence__init(tm_msgs__msg__SctResponse__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__SctResponse * data = NULL;

  if (size) {
    data = (tm_msgs__msg__SctResponse *)allocator.zero_allocate(size, sizeof(tm_msgs__msg__SctResponse), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_msgs__msg__SctResponse__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_msgs__msg__SctResponse__fini(&data[i - 1]);
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
tm_msgs__msg__SctResponse__Sequence__fini(tm_msgs__msg__SctResponse__Sequence * array)
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
      tm_msgs__msg__SctResponse__fini(&array->data[i]);
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

tm_msgs__msg__SctResponse__Sequence *
tm_msgs__msg__SctResponse__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__SctResponse__Sequence * array = (tm_msgs__msg__SctResponse__Sequence *)allocator.allocate(sizeof(tm_msgs__msg__SctResponse__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_msgs__msg__SctResponse__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_msgs__msg__SctResponse__Sequence__destroy(tm_msgs__msg__SctResponse__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_msgs__msg__SctResponse__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_msgs__msg__SctResponse__Sequence__are_equal(const tm_msgs__msg__SctResponse__Sequence * lhs, const tm_msgs__msg__SctResponse__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_msgs__msg__SctResponse__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_msgs__msg__SctResponse__Sequence__copy(
  const tm_msgs__msg__SctResponse__Sequence * input,
  tm_msgs__msg__SctResponse__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_msgs__msg__SctResponse);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_msgs__msg__SctResponse * data =
      (tm_msgs__msg__SctResponse *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_msgs__msg__SctResponse__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_msgs__msg__SctResponse__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_msgs__msg__SctResponse__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
