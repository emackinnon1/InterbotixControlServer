// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice

#ifndef INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__BUILDER_HPP_
#define INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interbotix_rpi_msgs/msg/detail/pixel_commands__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interbotix_rpi_msgs
{

namespace msg
{

namespace builder
{

class Init_PixelCommands_iterations
{
public:
  explicit Init_PixelCommands_iterations(::interbotix_rpi_msgs::msg::PixelCommands & msg)
  : msg_(msg)
  {}
  ::interbotix_rpi_msgs::msg::PixelCommands iterations(::interbotix_rpi_msgs::msg::PixelCommands::_iterations_type arg)
  {
    msg_.iterations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interbotix_rpi_msgs::msg::PixelCommands msg_;
};

class Init_PixelCommands_period
{
public:
  explicit Init_PixelCommands_period(::interbotix_rpi_msgs::msg::PixelCommands & msg)
  : msg_(msg)
  {}
  Init_PixelCommands_iterations period(::interbotix_rpi_msgs::msg::PixelCommands::_period_type arg)
  {
    msg_.period = std::move(arg);
    return Init_PixelCommands_iterations(msg_);
  }

private:
  ::interbotix_rpi_msgs::msg::PixelCommands msg_;
};

class Init_PixelCommands_brightness
{
public:
  explicit Init_PixelCommands_brightness(::interbotix_rpi_msgs::msg::PixelCommands & msg)
  : msg_(msg)
  {}
  Init_PixelCommands_period brightness(::interbotix_rpi_msgs::msg::PixelCommands::_brightness_type arg)
  {
    msg_.brightness = std::move(arg);
    return Init_PixelCommands_period(msg_);
  }

private:
  ::interbotix_rpi_msgs::msg::PixelCommands msg_;
};

class Init_PixelCommands_color
{
public:
  explicit Init_PixelCommands_color(::interbotix_rpi_msgs::msg::PixelCommands & msg)
  : msg_(msg)
  {}
  Init_PixelCommands_brightness color(::interbotix_rpi_msgs::msg::PixelCommands::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_PixelCommands_brightness(msg_);
  }

private:
  ::interbotix_rpi_msgs::msg::PixelCommands msg_;
};

class Init_PixelCommands_pixel
{
public:
  explicit Init_PixelCommands_pixel(::interbotix_rpi_msgs::msg::PixelCommands & msg)
  : msg_(msg)
  {}
  Init_PixelCommands_color pixel(::interbotix_rpi_msgs::msg::PixelCommands::_pixel_type arg)
  {
    msg_.pixel = std::move(arg);
    return Init_PixelCommands_color(msg_);
  }

private:
  ::interbotix_rpi_msgs::msg::PixelCommands msg_;
};

class Init_PixelCommands_set_all_leds
{
public:
  explicit Init_PixelCommands_set_all_leds(::interbotix_rpi_msgs::msg::PixelCommands & msg)
  : msg_(msg)
  {}
  Init_PixelCommands_pixel set_all_leds(::interbotix_rpi_msgs::msg::PixelCommands::_set_all_leds_type arg)
  {
    msg_.set_all_leds = std::move(arg);
    return Init_PixelCommands_pixel(msg_);
  }

private:
  ::interbotix_rpi_msgs::msg::PixelCommands msg_;
};

class Init_PixelCommands_cmd_type
{
public:
  Init_PixelCommands_cmd_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PixelCommands_set_all_leds cmd_type(::interbotix_rpi_msgs::msg::PixelCommands::_cmd_type_type arg)
  {
    msg_.cmd_type = std::move(arg);
    return Init_PixelCommands_set_all_leds(msg_);
  }

private:
  ::interbotix_rpi_msgs::msg::PixelCommands msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interbotix_rpi_msgs::msg::PixelCommands>()
{
  return interbotix_rpi_msgs::msg::builder::Init_PixelCommands_cmd_type();
}

}  // namespace interbotix_rpi_msgs

#endif  // INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__BUILDER_HPP_
