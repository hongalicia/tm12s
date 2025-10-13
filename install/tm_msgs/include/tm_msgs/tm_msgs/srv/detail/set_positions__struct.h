// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tm_msgs:srv/SetPositions.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__SRV__DETAIL__SET_POSITIONS__STRUCT_H_
#define TM_MSGS__SRV__DETAIL__SET_POSITIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PTP_J'.
enum
{
  tm_msgs__srv__SetPositions_Request__PTP_J = 1
};

/// Constant 'PTP_T'.
enum
{
  tm_msgs__srv__SetPositions_Request__PTP_T = 2
};

/// Constant 'LINE_T'.
/**
  * int8 LINE_J = 3
 */
enum
{
  tm_msgs__srv__SetPositions_Request__LINE_T = 4
};

// Include directives for member types
// Member 'positions'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/SetPositions in the package tm_msgs.
typedef struct tm_msgs__srv__SetPositions_Request
{
  /// int8 CIRC_J = 5
  /// int8 CIRC_T = 6
  /// int8 PLINE_J = 7
  /// int8 PLINE_T = 8
  int8_t motion_type;
  rosidl_runtime_c__double__Sequence positions;
  /// motion velocity: if expressed in Cartesian coordinate (unit: m/s), if expressed in joint velocity (unit: rad/s, and the maximum value is limited to pi)
  double velocity;
  /// time to reach maximum speed (unit: ms)
  double acc_time;
  /// blending value: expressed as a percentage (unit: %, and the minimum value of 0 means no blending)
  int32_t blend_percentage;
  /// precise position mode : If activated, the amount of error in the final position will converge more.
  bool fine_goal;
} tm_msgs__srv__SetPositions_Request;

// Struct for a sequence of tm_msgs__srv__SetPositions_Request.
typedef struct tm_msgs__srv__SetPositions_Request__Sequence
{
  tm_msgs__srv__SetPositions_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tm_msgs__srv__SetPositions_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetPositions in the package tm_msgs.
typedef struct tm_msgs__srv__SetPositions_Response
{
  bool ok;
} tm_msgs__srv__SetPositions_Response;

// Struct for a sequence of tm_msgs__srv__SetPositions_Response.
typedef struct tm_msgs__srv__SetPositions_Response__Sequence
{
  tm_msgs__srv__SetPositions_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tm_msgs__srv__SetPositions_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TM_MSGS__SRV__DETAIL__SET_POSITIONS__STRUCT_H_
