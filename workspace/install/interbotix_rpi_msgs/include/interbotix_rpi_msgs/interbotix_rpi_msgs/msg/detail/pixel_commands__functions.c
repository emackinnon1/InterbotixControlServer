// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `cmd_type`
#include "rosidl_runtime_c/string_functions.h"

bool
interbotix_rpi_msgs__msg__PixelCommands__init(interbotix_rpi_msgs__msg__PixelCommands * msg)
{
  if (!msg) {
    return false;
  }
  // cmd_type
  if (!rosidl_runtime_c__String__init(&msg->cmd_type)) {
    interbotix_rpi_msgs__msg__PixelCommands__fini(msg);
    return false;
  }
  // set_all_leds
  // pixel
  // color
  // brightness
  // period
  // iterations
  return true;
}

void
interbotix_rpi_msgs__msg__PixelCommands__fini(interbotix_rpi_msgs__msg__PixelCommands * msg)
{
  if (!msg) {
    return;
  }
  // cmd_type
  rosidl_runtime_c__String__fini(&msg->cmd_type);
  // set_all_leds
  // pixel
  // color
  // brightness
  // period
  // iterations
}

bool
interbotix_rpi_msgs__msg__PixelCommands__are_equal(const interbotix_rpi_msgs__msg__PixelCommands * lhs, const interbotix_rpi_msgs__msg__PixelCommands * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cmd_type), &(rhs->cmd_type)))
  {
    return false;
  }
  // set_all_leds
  if (lhs->set_all_leds != rhs->set_all_leds) {
    return false;
  }
  // pixel
  if (lhs->pixel != rhs->pixel) {
    return false;
  }
  // color
  if (lhs->color != rhs->color) {
    return false;
  }
  // brightness
  if (lhs->brightness != rhs->brightness) {
    return false;
  }
  // period
  if (lhs->period != rhs->period) {
    return false;
  }
  // iterations
  if (lhs->iterations != rhs->iterations) {
    return false;
  }
  return true;
}

bool
interbotix_rpi_msgs__msg__PixelCommands__copy(
  const interbotix_rpi_msgs__msg__PixelCommands * input,
  interbotix_rpi_msgs__msg__PixelCommands * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd_type
  if (!rosidl_runtime_c__String__copy(
      &(input->cmd_type), &(output->cmd_type)))
  {
    return false;
  }
  // set_all_leds
  output->set_all_leds = input->set_all_leds;
  // pixel
  output->pixel = input->pixel;
  // color
  output->color = input->color;
  // brightness
  output->brightness = input->brightness;
  // period
  output->period = input->period;
  // iterations
  output->iterations = input->iterations;
  return true;
}

interbotix_rpi_msgs__msg__PixelCommands *
interbotix_rpi_msgs__msg__PixelCommands__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interbotix_rpi_msgs__msg__PixelCommands * msg = (interbotix_rpi_msgs__msg__PixelCommands *)allocator.allocate(sizeof(interbotix_rpi_msgs__msg__PixelCommands), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interbotix_rpi_msgs__msg__PixelCommands));
  bool success = interbotix_rpi_msgs__msg__PixelCommands__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interbotix_rpi_msgs__msg__PixelCommands__destroy(interbotix_rpi_msgs__msg__PixelCommands * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interbotix_rpi_msgs__msg__PixelCommands__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interbotix_rpi_msgs__msg__PixelCommands__Sequence__init(interbotix_rpi_msgs__msg__PixelCommands__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interbotix_rpi_msgs__msg__PixelCommands * data = NULL;

  if (size) {
    data = (interbotix_rpi_msgs__msg__PixelCommands *)allocator.zero_allocate(size, sizeof(interbotix_rpi_msgs__msg__PixelCommands), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interbotix_rpi_msgs__msg__PixelCommands__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interbotix_rpi_msgs__msg__PixelCommands__fini(&data[i - 1]);
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
interbotix_rpi_msgs__msg__PixelCommands__Sequence__fini(interbotix_rpi_msgs__msg__PixelCommands__Sequence * array)
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
      interbotix_rpi_msgs__msg__PixelCommands__fini(&array->data[i]);
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

interbotix_rpi_msgs__msg__PixelCommands__Sequence *
interbotix_rpi_msgs__msg__PixelCommands__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interbotix_rpi_msgs__msg__PixelCommands__Sequence * array = (interbotix_rpi_msgs__msg__PixelCommands__Sequence *)allocator.allocate(sizeof(interbotix_rpi_msgs__msg__PixelCommands__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interbotix_rpi_msgs__msg__PixelCommands__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interbotix_rpi_msgs__msg__PixelCommands__Sequence__destroy(interbotix_rpi_msgs__msg__PixelCommands__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interbotix_rpi_msgs__msg__PixelCommands__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interbotix_rpi_msgs__msg__PixelCommands__Sequence__are_equal(const interbotix_rpi_msgs__msg__PixelCommands__Sequence * lhs, const interbotix_rpi_msgs__msg__PixelCommands__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interbotix_rpi_msgs__msg__PixelCommands__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interbotix_rpi_msgs__msg__PixelCommands__Sequence__copy(
  const interbotix_rpi_msgs__msg__PixelCommands__Sequence * input,
  interbotix_rpi_msgs__msg__PixelCommands__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interbotix_rpi_msgs__msg__PixelCommands);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interbotix_rpi_msgs__msg__PixelCommands * data =
      (interbotix_rpi_msgs__msg__PixelCommands *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interbotix_rpi_msgs__msg__PixelCommands__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interbotix_rpi_msgs__msg__PixelCommands__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interbotix_rpi_msgs__msg__PixelCommands__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
