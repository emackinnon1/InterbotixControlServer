// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice

#ifndef INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__TRAITS_HPP_
#define INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interbotix_rpi_msgs/msg/detail/pixel_commands__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interbotix_rpi_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PixelCommands & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd_type
  {
    out << "cmd_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_type, out);
    out << ", ";
  }

  // member: set_all_leds
  {
    out << "set_all_leds: ";
    rosidl_generator_traits::value_to_yaml(msg.set_all_leds, out);
    out << ", ";
  }

  // member: pixel
  {
    out << "pixel: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << ", ";
  }

  // member: brightness
  {
    out << "brightness: ";
    rosidl_generator_traits::value_to_yaml(msg.brightness, out);
    out << ", ";
  }

  // member: period
  {
    out << "period: ";
    rosidl_generator_traits::value_to_yaml(msg.period, out);
    out << ", ";
  }

  // member: iterations
  {
    out << "iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.iterations, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PixelCommands & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_type, out);
    out << "\n";
  }

  // member: set_all_leds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "set_all_leds: ";
    rosidl_generator_traits::value_to_yaml(msg.set_all_leds, out);
    out << "\n";
  }

  // member: pixel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pixel: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }

  // member: brightness
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brightness: ";
    rosidl_generator_traits::value_to_yaml(msg.brightness, out);
    out << "\n";
  }

  // member: period
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "period: ";
    rosidl_generator_traits::value_to_yaml(msg.period, out);
    out << "\n";
  }

  // member: iterations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.iterations, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PixelCommands & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interbotix_rpi_msgs

namespace rosidl_generator_traits
{

[[deprecated("use interbotix_rpi_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interbotix_rpi_msgs::msg::PixelCommands & msg,
  std::ostream & out, size_t indentation = 0)
{
  interbotix_rpi_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interbotix_rpi_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const interbotix_rpi_msgs::msg::PixelCommands & msg)
{
  return interbotix_rpi_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interbotix_rpi_msgs::msg::PixelCommands>()
{
  return "interbotix_rpi_msgs::msg::PixelCommands";
}

template<>
inline const char * name<interbotix_rpi_msgs::msg::PixelCommands>()
{
  return "interbotix_rpi_msgs/msg/PixelCommands";
}

template<>
struct has_fixed_size<interbotix_rpi_msgs::msg::PixelCommands>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interbotix_rpi_msgs::msg::PixelCommands>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interbotix_rpi_msgs::msg::PixelCommands>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__TRAITS_HPP_
