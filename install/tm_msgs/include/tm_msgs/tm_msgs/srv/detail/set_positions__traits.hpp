// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tm_msgs:srv/SetPositions.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__SRV__DETAIL__SET_POSITIONS__TRAITS_HPP_
#define TM_MSGS__SRV__DETAIL__SET_POSITIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tm_msgs/srv/detail/set_positions__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tm_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPositions_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: motion_type
  {
    out << "motion_type: ";
    rosidl_generator_traits::value_to_yaml(msg.motion_type, out);
    out << ", ";
  }

  // member: positions
  {
    if (msg.positions.size() == 0) {
      out << "positions: []";
    } else {
      out << "positions: [";
      size_t pending_items = msg.positions.size();
      for (auto item : msg.positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: acc_time
  {
    out << "acc_time: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_time, out);
    out << ", ";
  }

  // member: blend_percentage
  {
    out << "blend_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.blend_percentage, out);
    out << ", ";
  }

  // member: fine_goal
  {
    out << "fine_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.fine_goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPositions_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motion_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motion_type: ";
    rosidl_generator_traits::value_to_yaml(msg.motion_type, out);
    out << "\n";
  }

  // member: positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positions.size() == 0) {
      out << "positions: []\n";
    } else {
      out << "positions:\n";
      for (auto item : msg.positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: acc_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_time: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_time, out);
    out << "\n";
  }

  // member: blend_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "blend_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.blend_percentage, out);
    out << "\n";
  }

  // member: fine_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fine_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.fine_goal, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPositions_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tm_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tm_msgs::srv::SetPositions_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::srv::SetPositions_Request & msg)
{
  return tm_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::srv::SetPositions_Request>()
{
  return "tm_msgs::srv::SetPositions_Request";
}

template<>
inline const char * name<tm_msgs::srv::SetPositions_Request>()
{
  return "tm_msgs/srv/SetPositions_Request";
}

template<>
struct has_fixed_size<tm_msgs::srv::SetPositions_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_msgs::srv::SetPositions_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_msgs::srv::SetPositions_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace tm_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPositions_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: ok
  {
    out << "ok: ";
    rosidl_generator_traits::value_to_yaml(msg.ok, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPositions_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ok: ";
    rosidl_generator_traits::value_to_yaml(msg.ok, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPositions_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tm_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tm_msgs::srv::SetPositions_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::srv::SetPositions_Response & msg)
{
  return tm_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::srv::SetPositions_Response>()
{
  return "tm_msgs::srv::SetPositions_Response";
}

template<>
inline const char * name<tm_msgs::srv::SetPositions_Response>()
{
  return "tm_msgs/srv/SetPositions_Response";
}

template<>
struct has_fixed_size<tm_msgs::srv::SetPositions_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tm_msgs::srv::SetPositions_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tm_msgs::srv::SetPositions_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tm_msgs::srv::SetPositions>()
{
  return "tm_msgs::srv::SetPositions";
}

template<>
inline const char * name<tm_msgs::srv::SetPositions>()
{
  return "tm_msgs/srv/SetPositions";
}

template<>
struct has_fixed_size<tm_msgs::srv::SetPositions>
  : std::integral_constant<
    bool,
    has_fixed_size<tm_msgs::srv::SetPositions_Request>::value &&
    has_fixed_size<tm_msgs::srv::SetPositions_Response>::value
  >
{
};

template<>
struct has_bounded_size<tm_msgs::srv::SetPositions>
  : std::integral_constant<
    bool,
    has_bounded_size<tm_msgs::srv::SetPositions_Request>::value &&
    has_bounded_size<tm_msgs::srv::SetPositions_Response>::value
  >
{
};

template<>
struct is_service<tm_msgs::srv::SetPositions>
  : std::true_type
{
};

template<>
struct is_service_request<tm_msgs::srv::SetPositions_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tm_msgs::srv::SetPositions_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TM_MSGS__SRV__DETAIL__SET_POSITIONS__TRAITS_HPP_
