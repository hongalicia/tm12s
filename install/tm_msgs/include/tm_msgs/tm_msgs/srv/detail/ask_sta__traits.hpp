// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tm_msgs:srv/AskSta.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__SRV__DETAIL__ASK_STA__TRAITS_HPP_
#define TM_MSGS__SRV__DETAIL__ASK_STA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tm_msgs/srv/detail/ask_sta__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tm_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AskSta_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: subcmd
  {
    out << "subcmd: ";
    rosidl_generator_traits::value_to_yaml(msg.subcmd, out);
    out << ", ";
  }

  // member: subdata
  {
    out << "subdata: ";
    rosidl_generator_traits::value_to_yaml(msg.subdata, out);
    out << ", ";
  }

  // member: wait_time
  {
    out << "wait_time: ";
    rosidl_generator_traits::value_to_yaml(msg.wait_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AskSta_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: subcmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "subcmd: ";
    rosidl_generator_traits::value_to_yaml(msg.subcmd, out);
    out << "\n";
  }

  // member: subdata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "subdata: ";
    rosidl_generator_traits::value_to_yaml(msg.subdata, out);
    out << "\n";
  }

  // member: wait_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wait_time: ";
    rosidl_generator_traits::value_to_yaml(msg.wait_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AskSta_Request & msg, bool use_flow_style = false)
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
  const tm_msgs::srv::AskSta_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::srv::AskSta_Request & msg)
{
  return tm_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::srv::AskSta_Request>()
{
  return "tm_msgs::srv::AskSta_Request";
}

template<>
inline const char * name<tm_msgs::srv::AskSta_Request>()
{
  return "tm_msgs/srv/AskSta_Request";
}

template<>
struct has_fixed_size<tm_msgs::srv::AskSta_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_msgs::srv::AskSta_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_msgs::srv::AskSta_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace tm_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AskSta_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: ok
  {
    out << "ok: ";
    rosidl_generator_traits::value_to_yaml(msg.ok, out);
    out << ", ";
  }

  // member: subcmd
  {
    out << "subcmd: ";
    rosidl_generator_traits::value_to_yaml(msg.subcmd, out);
    out << ", ";
  }

  // member: subdata
  {
    out << "subdata: ";
    rosidl_generator_traits::value_to_yaml(msg.subdata, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AskSta_Response & msg,
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

  // member: subcmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "subcmd: ";
    rosidl_generator_traits::value_to_yaml(msg.subcmd, out);
    out << "\n";
  }

  // member: subdata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "subdata: ";
    rosidl_generator_traits::value_to_yaml(msg.subdata, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AskSta_Response & msg, bool use_flow_style = false)
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
  const tm_msgs::srv::AskSta_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::srv::AskSta_Response & msg)
{
  return tm_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::srv::AskSta_Response>()
{
  return "tm_msgs::srv::AskSta_Response";
}

template<>
inline const char * name<tm_msgs::srv::AskSta_Response>()
{
  return "tm_msgs/srv/AskSta_Response";
}

template<>
struct has_fixed_size<tm_msgs::srv::AskSta_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_msgs::srv::AskSta_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_msgs::srv::AskSta_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tm_msgs::srv::AskSta>()
{
  return "tm_msgs::srv::AskSta";
}

template<>
inline const char * name<tm_msgs::srv::AskSta>()
{
  return "tm_msgs/srv/AskSta";
}

template<>
struct has_fixed_size<tm_msgs::srv::AskSta>
  : std::integral_constant<
    bool,
    has_fixed_size<tm_msgs::srv::AskSta_Request>::value &&
    has_fixed_size<tm_msgs::srv::AskSta_Response>::value
  >
{
};

template<>
struct has_bounded_size<tm_msgs::srv::AskSta>
  : std::integral_constant<
    bool,
    has_bounded_size<tm_msgs::srv::AskSta_Request>::value &&
    has_bounded_size<tm_msgs::srv::AskSta_Response>::value
  >
{
};

template<>
struct is_service<tm_msgs::srv::AskSta>
  : std::true_type
{
};

template<>
struct is_service_request<tm_msgs::srv::AskSta_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tm_msgs::srv::AskSta_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TM_MSGS__SRV__DETAIL__ASK_STA__TRAITS_HPP_
