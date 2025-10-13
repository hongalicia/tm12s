// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tm_msgs:msg/SvrResponse.idl
// generated code does not contain a copyright notice
#include "tm_msgs/msg/detail/svr_response__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `id`
// Member `content`
#include "rosidl_runtime_c/string_functions.h"

bool
tm_msgs__msg__SvrResponse__init(tm_msgs__msg__SvrResponse * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tm_msgs__msg__SvrResponse__fini(msg);
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__init(&msg->id)) {
    tm_msgs__msg__SvrResponse__fini(msg);
    return false;
  }
  // mode
  // content
  if (!rosidl_runtime_c__String__init(&msg->content)) {
    tm_msgs__msg__SvrResponse__fini(msg);
    return false;
  }
  // error_code
  return true;
}

void
tm_msgs__msg__SvrResponse__fini(tm_msgs__msg__SvrResponse * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  rosidl_runtime_c__String__fini(&msg->id);
  // mode
  // content
  rosidl_runtime_c__String__fini(&msg->content);
  // error_code
}

bool
tm_msgs__msg__SvrResponse__are_equal(const tm_msgs__msg__SvrResponse * lhs, const tm_msgs__msg__SvrResponse * rhs)
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
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // content
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->content), &(rhs->content)))
  {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  return true;
}

bool
tm_msgs__msg__SvrResponse__copy(
  const tm_msgs__msg__SvrResponse * input,
  tm_msgs__msg__SvrResponse * output)
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
  // mode
  output->mode = input->mode;
  // content
  if (!rosidl_runtime_c__String__copy(
      &(input->content), &(output->content)))
  {
    return false;
  }
  // error_code
  output->error_code = input->error_code;
  return true;
}

tm_msgs__msg__SvrResponse *
tm_msgs__msg__SvrResponse__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__SvrResponse * msg = (tm_msgs__msg__SvrResponse *)allocator.allocate(sizeof(tm_msgs__msg__SvrResponse), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_msgs__msg__SvrResponse));
  bool success = tm_msgs__msg__SvrResponse__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_msgs__msg__SvrResponse__destroy(tm_msgs__msg__SvrResponse * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_msgs__msg__SvrResponse__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_msgs__msg__SvrResponse__Sequence__init(tm_msgs__msg__SvrResponse__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__SvrResponse * data = NULL;

  if (size) {
    data = (tm_msgs__msg__SvrResponse *)allocator.zero_allocate(size, sizeof(tm_msgs__msg__SvrResponse), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_msgs__msg__SvrResponse__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_msgs__msg__SvrResponse__fini(&data[i - 1]);
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
tm_msgs__msg__SvrResponse__Sequence__fini(tm_msgs__msg__SvrResponse__Sequence * array)
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
      tm_msgs__msg__SvrResponse__fini(&array->data[i]);
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

tm_msgs__msg__SvrResponse__Sequence *
tm_msgs__msg__SvrResponse__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__SvrResponse__Sequence * array = (tm_msgs__msg__SvrResponse__Sequence *)allocator.allocate(sizeof(tm_msgs__msg__SvrResponse__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_msgs__msg__SvrResponse__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_msgs__msg__SvrResponse__Sequence__destroy(tm_msgs__msg__SvrResponse__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_msgs__msg__SvrResponse__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_msgs__msg__SvrResponse__Sequence__are_equal(const tm_msgs__msg__SvrResponse__Sequence * lhs, const tm_msgs__msg__SvrResponse__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_msgs__msg__SvrResponse__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_msgs__msg__SvrResponse__Sequence__copy(
  const tm_msgs__msg__SvrResponse__Sequence * input,
  tm_msgs__msg__SvrResponse__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_msgs__msg__SvrResponse);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_msgs__msg__SvrResponse * data =
      (tm_msgs__msg__SvrResponse *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_msgs__msg__SvrResponse__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_msgs__msg__SvrResponse__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_msgs__msg__SvrResponse__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
