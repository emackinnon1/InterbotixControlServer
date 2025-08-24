// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice

#ifndef INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__STRUCT_HPP_
#define INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interbotix_rpi_msgs__msg__PixelCommands __attribute__((deprecated))
#else
# define DEPRECATED__interbotix_rpi_msgs__msg__PixelCommands __declspec(deprecated)
#endif

namespace interbotix_rpi_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PixelCommands_
{
  using Type = PixelCommands_<ContainerAllocator>;

  explicit PixelCommands_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd_type = "";
      this->set_all_leds = false;
      this->pixel = 0;
      this->color = 0ul;
      this->brightness = 0;
      this->period = 0ul;
      this->iterations = 0;
    }
  }

  explicit PixelCommands_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cmd_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd_type = "";
      this->set_all_leds = false;
      this->pixel = 0;
      this->color = 0ul;
      this->brightness = 0;
      this->period = 0ul;
      this->iterations = 0;
    }
  }

  // field types and members
  using _cmd_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cmd_type_type cmd_type;
  using _set_all_leds_type =
    bool;
  _set_all_leds_type set_all_leds;
  using _pixel_type =
    uint8_t;
  _pixel_type pixel;
  using _color_type =
    uint32_t;
  _color_type color;
  using _brightness_type =
    uint8_t;
  _brightness_type brightness;
  using _period_type =
    uint32_t;
  _period_type period;
  using _iterations_type =
    uint8_t;
  _iterations_type iterations;

  // setters for named parameter idiom
  Type & set__cmd_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cmd_type = _arg;
    return *this;
  }
  Type & set__set_all_leds(
    const bool & _arg)
  {
    this->set_all_leds = _arg;
    return *this;
  }
  Type & set__pixel(
    const uint8_t & _arg)
  {
    this->pixel = _arg;
    return *this;
  }
  Type & set__color(
    const uint32_t & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__brightness(
    const uint8_t & _arg)
  {
    this->brightness = _arg;
    return *this;
  }
  Type & set__period(
    const uint32_t & _arg)
  {
    this->period = _arg;
    return *this;
  }
  Type & set__iterations(
    const uint8_t & _arg)
  {
    this->iterations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator> *;
  using ConstRawPtr =
    const interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interbotix_rpi_msgs__msg__PixelCommands
    std::shared_ptr<interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interbotix_rpi_msgs__msg__PixelCommands
    std::shared_ptr<interbotix_rpi_msgs::msg::PixelCommands_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PixelCommands_ & other) const
  {
    if (this->cmd_type != other.cmd_type) {
      return false;
    }
    if (this->set_all_leds != other.set_all_leds) {
      return false;
    }
    if (this->pixel != other.pixel) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    if (this->brightness != other.brightness) {
      return false;
    }
    if (this->period != other.period) {
      return false;
    }
    if (this->iterations != other.iterations) {
      return false;
    }
    return true;
  }
  bool operator!=(const PixelCommands_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PixelCommands_

// alias to use template instance with default allocator
using PixelCommands =
  interbotix_rpi_msgs::msg::PixelCommands_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interbotix_rpi_msgs

#endif  // INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__STRUCT_HPP_
