// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice

#ifndef INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__STRUCT_H_
#define INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmd_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PixelCommands in the package interbotix_rpi_msgs.
/**
  * This message is used specifically in the interbotix_rpi_modules package
  *
  * Configure NeoPixel Leds on a Raspberry Pi
 */
typedef struct interbotix_rpi_msgs__msg__PixelCommands
{
  /// Either 'pulse', 'blink', 'brightness', or 'color'.
  /// Note that the 'pulse' and 'brightness' options will
  /// affect all Leds even if 'set_all_leds' is set to False.
  rosidl_runtime_c__String cmd_type;
  /// Set to True to configure all LEDs
  bool set_all_leds;
  /// Index of NeoPixel to change (only considered if 'set_all_leds' is false)
  uint8_t pixel;
  /// Value indicates Color to be used (HEX - RGB)
  uint32_t color;
  /// Value indicates LED brightness level (0-255)
  uint8_t brightness;
  /// time delay for blink or pulse (milliseconds)
  uint32_t period;
  /// iterations = number of iterations for blink or pulse
  uint8_t iterations;
} interbotix_rpi_msgs__msg__PixelCommands;

// Struct for a sequence of interbotix_rpi_msgs__msg__PixelCommands.
typedef struct interbotix_rpi_msgs__msg__PixelCommands__Sequence
{
  interbotix_rpi_msgs__msg__PixelCommands * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interbotix_rpi_msgs__msg__PixelCommands__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__STRUCT_H_
