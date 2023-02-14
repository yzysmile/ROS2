// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tutorial_interfaces:msg/Num.idl
// generated code does not contain a copyright notice
#include "tutorial_interfaces/msg/detail/num__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
tutorial_interfaces__msg__Num__init(tutorial_interfaces__msg__Num * msg)
{
  if (!msg) {
    return false;
  }
  // num
  return true;
}

void
tutorial_interfaces__msg__Num__fini(tutorial_interfaces__msg__Num * msg)
{
  if (!msg) {
    return;
  }
  // num
}

bool
tutorial_interfaces__msg__Num__are_equal(const tutorial_interfaces__msg__Num * lhs, const tutorial_interfaces__msg__Num * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num
  if (lhs->num != rhs->num) {
    return false;
  }
  return true;
}

bool
tutorial_interfaces__msg__Num__copy(
  const tutorial_interfaces__msg__Num * input,
  tutorial_interfaces__msg__Num * output)
{
  if (!input || !output) {
    return false;
  }
  // num
  output->num = input->num;
  return true;
}

tutorial_interfaces__msg__Num *
tutorial_interfaces__msg__Num__create()
{
  tutorial_interfaces__msg__Num * msg = (tutorial_interfaces__msg__Num *)malloc(sizeof(tutorial_interfaces__msg__Num));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tutorial_interfaces__msg__Num));
  bool success = tutorial_interfaces__msg__Num__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
tutorial_interfaces__msg__Num__destroy(tutorial_interfaces__msg__Num * msg)
{
  if (msg) {
    tutorial_interfaces__msg__Num__fini(msg);
  }
  free(msg);
}


bool
tutorial_interfaces__msg__Num__Sequence__init(tutorial_interfaces__msg__Num__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  tutorial_interfaces__msg__Num * data = NULL;
  if (size) {
    data = (tutorial_interfaces__msg__Num *)calloc(size, sizeof(tutorial_interfaces__msg__Num));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tutorial_interfaces__msg__Num__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tutorial_interfaces__msg__Num__fini(&data[i - 1]);
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
tutorial_interfaces__msg__Num__Sequence__fini(tutorial_interfaces__msg__Num__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tutorial_interfaces__msg__Num__fini(&array->data[i]);
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

tutorial_interfaces__msg__Num__Sequence *
tutorial_interfaces__msg__Num__Sequence__create(size_t size)
{
  tutorial_interfaces__msg__Num__Sequence * array = (tutorial_interfaces__msg__Num__Sequence *)malloc(sizeof(tutorial_interfaces__msg__Num__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = tutorial_interfaces__msg__Num__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
tutorial_interfaces__msg__Num__Sequence__destroy(tutorial_interfaces__msg__Num__Sequence * array)
{
  if (array) {
    tutorial_interfaces__msg__Num__Sequence__fini(array);
  }
  free(array);
}

bool
tutorial_interfaces__msg__Num__Sequence__are_equal(const tutorial_interfaces__msg__Num__Sequence * lhs, const tutorial_interfaces__msg__Num__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tutorial_interfaces__msg__Num__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tutorial_interfaces__msg__Num__Sequence__copy(
  const tutorial_interfaces__msg__Num__Sequence * input,
  tutorial_interfaces__msg__Num__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tutorial_interfaces__msg__Num);
    tutorial_interfaces__msg__Num * data =
      (tutorial_interfaces__msg__Num *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tutorial_interfaces__msg__Num__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          tutorial_interfaces__msg__Num__fini(&data[i]);
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
    if (!tutorial_interfaces__msg__Num__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
