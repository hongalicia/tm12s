// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tm_msgs:msg/FeedbackState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tm_msgs/msg/detail/feedback_state__rosidl_typesupport_introspection_c.h"
#include "tm_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tm_msgs/msg/detail/feedback_state__functions.h"
#include "tm_msgs/msg/detail/feedback_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `joint_pos`
// Member `joint_vel`
// Member `joint_tor`
// Member `tool0_pose`
// Member `tool_pose`
// Member `tcp_speed`
// Member `tcp_force`
// Member `joint_tor_average`
// Member `joint_tor_min`
// Member `joint_tor_max`
// Member `cb_digital_output`
// Member `cb_digital_input`
// Member `cb_analog_output`
// Member `cb_analog_input`
// Member `ee_digital_output`
// Member `ee_digital_input`
// Member `ee_analog_output`
// Member `ee_analog_input`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `error_content`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tm_msgs__msg__FeedbackState__init(message_memory);
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_fini_function(void * message_memory)
{
  tm_msgs__msg__FeedbackState__fini(message_memory);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_pos(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_pos(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_pos(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_pos(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_pos(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_pos(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_vel(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_vel(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_vel(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_vel(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_vel(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_vel(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_tor(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_tor(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_tor(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_tor(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__tool0_pose(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tool0_pose(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tool0_pose(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__tool0_pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tool0_pose(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__tool0_pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tool0_pose(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__tool0_pose(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__tool_pose(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tool_pose(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tool_pose(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__tool_pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tool_pose(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__tool_pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tool_pose(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__tool_pose(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__tcp_speed(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tcp_speed(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tcp_speed(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__tcp_speed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tcp_speed(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__tcp_speed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tcp_speed(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__tcp_speed(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__tcp_force(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tcp_force(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tcp_force(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__tcp_force(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tcp_force(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__tcp_force(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tcp_force(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__tcp_force(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_tor_average(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_average(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_average(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_tor_average(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_average(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_tor_average(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_average(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_tor_average(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_tor_min(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_min(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_min(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_tor_min(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_min(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_tor_min(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_min(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_tor_min(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_tor_max(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_max(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_max(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_tor_max(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_max(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_tor_max(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_max(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_tor_max(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__cb_digital_output(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_digital_output(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_digital_output(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__cb_digital_output(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_digital_output(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__cb_digital_output(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_digital_output(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__cb_digital_output(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__cb_digital_input(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_digital_input(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_digital_input(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__cb_digital_input(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_digital_input(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__cb_digital_input(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_digital_input(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__cb_digital_input(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__cb_analog_output(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_analog_output(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_analog_output(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__cb_analog_output(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_analog_output(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__cb_analog_output(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_analog_output(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__cb_analog_output(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__cb_analog_input(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_analog_input(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_analog_input(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__cb_analog_input(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_analog_input(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__cb_analog_input(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_analog_input(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__cb_analog_input(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__ee_digital_output(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_digital_output(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_digital_output(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__ee_digital_output(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_digital_output(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__ee_digital_output(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_digital_output(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__ee_digital_output(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__ee_digital_input(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_digital_input(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_digital_input(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__ee_digital_input(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_digital_input(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__ee_digital_input(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_digital_input(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__ee_digital_input(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__ee_analog_output(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_analog_output(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_analog_output(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__ee_analog_output(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_analog_output(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__ee_analog_output(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_analog_output(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__ee_analog_output(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__ee_analog_input(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_analog_input(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_analog_input(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__ee_analog_input(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_analog_input(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__ee_analog_input(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_analog_input(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__ee_analog_input(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_member_array[40] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_svr_connected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, is_svr_connected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_sct_connected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, is_sct_connected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tmsrv_cperr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, tmsrv_cperr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tmscript_cperr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, tmscript_cperr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tmsrv_dataerr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, tmsrv_dataerr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tmscript_dataerr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, tmscript_dataerr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_not_connect_in_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, max_not_connect_in_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "disconnection_times",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, disconnection_times),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, joint_pos),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_pos,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_pos,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_pos,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_pos,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_pos,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_pos  // resize(index) function pointer
  },
  {
    "joint_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, joint_vel),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_vel,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_vel,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_vel,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_vel,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_vel,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_vel  // resize(index) function pointer
  },
  {
    "joint_tor",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, joint_tor),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_tor,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_tor,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_tor,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_tor  // resize(index) function pointer
  },
  {
    "tool0_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, tool0_pose),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__tool0_pose,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tool0_pose,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tool0_pose,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__tool0_pose,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__tool0_pose,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__tool0_pose  // resize(index) function pointer
  },
  {
    "tool_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, tool_pose),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__tool_pose,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tool_pose,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tool_pose,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__tool_pose,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__tool_pose,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__tool_pose  // resize(index) function pointer
  },
  {
    "tcp_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, tcp_speed),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__tcp_speed,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tcp_speed,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tcp_speed,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__tcp_speed,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__tcp_speed,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__tcp_speed  // resize(index) function pointer
  },
  {
    "tcp_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, tcp_force),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__tcp_force,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__tcp_force,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__tcp_force,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__tcp_force,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__tcp_force,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__tcp_force  // resize(index) function pointer
  },
  {
    "joint_tor_average",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, joint_tor_average),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_tor_average,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_average,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_average,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_tor_average,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_tor_average,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_tor_average  // resize(index) function pointer
  },
  {
    "joint_tor_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, joint_tor_min),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_tor_min,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_min,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_min,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_tor_min,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_tor_min,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_tor_min  // resize(index) function pointer
  },
  {
    "joint_tor_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, joint_tor_max),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__joint_tor_max,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__joint_tor_max,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__joint_tor_max,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__joint_tor_max,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__joint_tor_max,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__joint_tor_max  // resize(index) function pointer
  },
  {
    "robot_link",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, robot_link),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_data_table_correct",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, is_data_table_correct),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, robot_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "project_run",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, project_run),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "project_pause",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, project_pause),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safetyguard_a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, safetyguard_a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "e_stop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, e_stop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "camera_light",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, camera_light),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "project_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, project_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ma_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, ma_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_light",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, robot_light),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cb_digital_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, cb_digital_output),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__cb_digital_output,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_digital_output,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_digital_output,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__cb_digital_output,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__cb_digital_output,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__cb_digital_output  // resize(index) function pointer
  },
  {
    "cb_digital_input",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, cb_digital_input),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__cb_digital_input,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_digital_input,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_digital_input,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__cb_digital_input,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__cb_digital_input,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__cb_digital_input  // resize(index) function pointer
  },
  {
    "cb_analog_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, cb_analog_output),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__cb_analog_output,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_analog_output,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_analog_output,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__cb_analog_output,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__cb_analog_output,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__cb_analog_output  // resize(index) function pointer
  },
  {
    "cb_analog_input",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, cb_analog_input),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__cb_analog_input,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__cb_analog_input,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__cb_analog_input,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__cb_analog_input,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__cb_analog_input,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__cb_analog_input  // resize(index) function pointer
  },
  {
    "ee_digital_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, ee_digital_output),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__ee_digital_output,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_digital_output,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_digital_output,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__ee_digital_output,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__ee_digital_output,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__ee_digital_output  // resize(index) function pointer
  },
  {
    "ee_digital_input",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, ee_digital_input),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__ee_digital_input,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_digital_input,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_digital_input,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__ee_digital_input,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__ee_digital_input,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__ee_digital_input  // resize(index) function pointer
  },
  {
    "ee_analog_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, ee_analog_output),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__ee_analog_output,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_analog_output,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_analog_output,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__ee_analog_output,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__ee_analog_output,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__ee_analog_output  // resize(index) function pointer
  },
  {
    "ee_analog_input",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, ee_analog_input),  // bytes offset in struct
    NULL,  // default value
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__size_function__FeedbackState__ee_analog_input,  // size() function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_const_function__FeedbackState__ee_analog_input,  // get_const(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__get_function__FeedbackState__ee_analog_input,  // get(index) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__fetch_function__FeedbackState__ee_analog_input,  // fetch(index, &value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__assign_function__FeedbackState__ee_analog_input,  // assign(index, value) function pointer
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__resize_function__FeedbackState__ee_analog_input  // resize(index) function pointer
  },
  {
    "error_content",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_msgs__msg__FeedbackState, error_content),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_members = {
  "tm_msgs__msg",  // message namespace
  "FeedbackState",  // message name
  40,  // number of fields
  sizeof(tm_msgs__msg__FeedbackState),
  tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_member_array,  // message members
  tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_init_function,  // function to initialize message memory (memory has to be allocated)
  tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_type_support_handle = {
  0,
  &tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tm_msgs, msg, FeedbackState)() {
  tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_type_support_handle.typesupport_identifier) {
    tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tm_msgs__msg__FeedbackState__rosidl_typesupport_introspection_c__FeedbackState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
