// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice

#ifndef INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__FUNCTIONS_H_
#define INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "interbotix_rpi_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "interbotix_rpi_msgs/msg/detail/pixel_commands__struct.h"

/// Initialize msg/PixelCommands message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interbotix_rpi_msgs__msg__PixelCommands
 * )) before or use
 * interbotix_rpi_msgs__msg__PixelCommands__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
bool
interbotix_rpi_msgs__msg__PixelCommands__init(interbotix_rpi_msgs__msg__PixelCommands * msg);

/// Finalize msg/PixelCommands message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
void
interbotix_rpi_msgs__msg__PixelCommands__fini(interbotix_rpi_msgs__msg__PixelCommands * msg);

/// Create msg/PixelCommands message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interbotix_rpi_msgs__msg__PixelCommands__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
interbotix_rpi_msgs__msg__PixelCommands *
interbotix_rpi_msgs__msg__PixelCommands__create();

/// Destroy msg/PixelCommands message.
/**
 * It calls
 * interbotix_rpi_msgs__msg__PixelCommands__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
void
interbotix_rpi_msgs__msg__PixelCommands__destroy(interbotix_rpi_msgs__msg__PixelCommands * msg);

/// Check for msg/PixelCommands message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
bool
interbotix_rpi_msgs__msg__PixelCommands__are_equal(const interbotix_rpi_msgs__msg__PixelCommands * lhs, const interbotix_rpi_msgs__msg__PixelCommands * rhs);

/// Copy a msg/PixelCommands message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
bool
interbotix_rpi_msgs__msg__PixelCommands__copy(
  const interbotix_rpi_msgs__msg__PixelCommands * input,
  interbotix_rpi_msgs__msg__PixelCommands * output);

/// Initialize array of msg/PixelCommands messages.
/**
 * It allocates the memory for the number of elements and calls
 * interbotix_rpi_msgs__msg__PixelCommands__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
bool
interbotix_rpi_msgs__msg__PixelCommands__Sequence__init(interbotix_rpi_msgs__msg__PixelCommands__Sequence * array, size_t size);

/// Finalize array of msg/PixelCommands messages.
/**
 * It calls
 * interbotix_rpi_msgs__msg__PixelCommands__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
void
interbotix_rpi_msgs__msg__PixelCommands__Sequence__fini(interbotix_rpi_msgs__msg__PixelCommands__Sequence * array);

/// Create array of msg/PixelCommands messages.
/**
 * It allocates the memory for the array and calls
 * interbotix_rpi_msgs__msg__PixelCommands__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
interbotix_rpi_msgs__msg__PixelCommands__Sequence *
interbotix_rpi_msgs__msg__PixelCommands__Sequence__create(size_t size);

/// Destroy array of msg/PixelCommands messages.
/**
 * It calls
 * interbotix_rpi_msgs__msg__PixelCommands__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
void
interbotix_rpi_msgs__msg__PixelCommands__Sequence__destroy(interbotix_rpi_msgs__msg__PixelCommands__Sequence * array);

/// Check for msg/PixelCommands message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
bool
interbotix_rpi_msgs__msg__PixelCommands__Sequence__are_equal(const interbotix_rpi_msgs__msg__PixelCommands__Sequence * lhs, const interbotix_rpi_msgs__msg__PixelCommands__Sequence * rhs);

/// Copy an array of msg/PixelCommands messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interbotix_rpi_msgs
bool
interbotix_rpi_msgs__msg__PixelCommands__Sequence__copy(
  const interbotix_rpi_msgs__msg__PixelCommands__Sequence * input,
  interbotix_rpi_msgs__msg__PixelCommands__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // INTERBOTIX_RPI_MSGS__MSG__DETAIL__PIXEL_COMMANDS__FUNCTIONS_H_
