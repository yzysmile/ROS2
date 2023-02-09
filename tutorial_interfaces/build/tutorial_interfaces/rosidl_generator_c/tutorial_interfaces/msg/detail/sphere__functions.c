// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tutorial_interfaces:msg/Sphere.idl
// generated code does not contain a copyright notice
#include "tutorial_interfaces/msg/detail/sphere__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `center`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
tutorial_interfaces__msg__Sphere__init(tutorial_interfaces__msg__Sphere * msg)
{
  if (!msg) {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__init(&msg->center)) {
    tutorial_interfaces__msg__Sphere__fini(msg);
    return false;
  }
  // radius
  return true;
}

void
tutorial_interfaces__msg__Sphere__fini(tutorial_interfaces__msg__Sphere * msg)
{
  if (!msg) {
    return;
  }
  // center
  geometry_msgs__msg__Point__fini(&msg->center);
  // radius
}

bool
tutorial_interfaces__msg__Sphere__are_equal(const tutorial_interfaces__msg__Sphere * lhs, const tutorial_interfaces__msg__Sphere * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->center), &(rhs->center)))
  {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  return true;
}

bool
tutorial_interfaces__msg__Sphere__copy(
  const tutorial_interfaces__msg__Sphere * input,
  tutorial_interfaces__msg__Sphere * output)
{
  if (!input || !output) {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__copy(
      &(input->center), &(output->center)))
  {
    return false;
  }
  // radius
  output->radius = input->radius;
  return true;
}

tutorial_interfaces__msg__Sphere *
tutorial_interfaces__msg__Sphere__create()
{
  tutorial_interfaces__msg__Sphere * msg = (tutorial_interfaces__msg__Sphere *)malloc(sizeof(tutorial_interfaces__msg__Sphere));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tutorial_interfaces__msg__Sphere));
  bool success = tutorial_interfaces__msg__Sphere__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
tutorial_interfaces__msg__Sphere__destroy(tutorial_interfaces__msg__Sphere * msg)
{
  if (msg) {
    tutorial_interfaces__msg__Sphere__fini(msg);
  }
  free(msg);
}


bool
tutorial_interfaces__msg__Sphere__Sequence__init(tutorial_interfaces__msg__Sphere__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  tutorial_interfaces__msg__Sphere * data = NULL;
  if (size) {
    data = (tutorial_interfaces__msg__Sphere *)calloc(size, sizeof(tutorial_interfaces__msg__Sphere));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tutorial_interfaces__msg__Sphere__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tutorial_interfaces__msg__Sphere__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tutorial_interfaces__msg__Sphere__Sequence__fini(tutorial_interfaces__msg__Sphere__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tutorial_interfaces__msg__Sphere__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tutorial_interfaces__msg__Sphere__Sequence *
tutorial_interfaces__msg__Sphere__Sequence__create(size_t size)
{
  tutorial_interfaces__msg__Sphere__Sequence * array = (tutorial_interfaces__msg__Sphere__Sequence *)malloc(sizeof(tutorial_interfaces__msg__Sphere__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = tutorial_interfaces__msg__Sphere__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
tutorial_interfaces__msg__Sphere__Sequence__destroy(tutorial_interfaces__msg__Sphere__Sequence * array)
{
  if (array) {
    tutorial_interfaces__msg__Sphere__Sequence__fini(array);
  }
  free(array);
}

bool
tutorial_interfaces__msg__Sphere__Sequence__are_equal(const tutorial_interfaces__msg__Sphere__Sequence * lhs, const tutorial_interfaces__msg__Sphere__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tutorial_interfaces__msg__Sphere__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tutorial_interfaces__msg__Sphere__Sequence__copy(
  const tutorial_interfaces__msg__Sphere__Sequence * input,
  tutorial_interfaces__msg__Sphere__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tutorial_interfaces__msg__Sphere);
    tutorial_interfaces__msg__Sphere * data =
      (tutorial_interfaces__msg__Sphere *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tutorial_interfaces__msg__Sphere__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          tutorial_interfaces__msg__Sphere__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tutorial_interfaces__msg__Sphere__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
