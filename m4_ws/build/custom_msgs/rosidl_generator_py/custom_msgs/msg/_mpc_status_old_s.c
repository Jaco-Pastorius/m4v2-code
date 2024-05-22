// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from custom_msgs:msg/MPCStatusOld.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "custom_msgs/msg/detail/mpc_status_old__struct.h"
#include "custom_msgs/msg/detail/mpc_status_old__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool custom_msgs__msg__mpc_status_old__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("custom_msgs.msg._mpc_status_old.MPCStatusOld", full_classname_dest, 44) == 0);
  }
  custom_msgs__msg__MPCStatusOld * ros_message = _ros_message;
  {  // timestamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "timestamp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timestamp = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // comptime
    PyObject * field = PyObject_GetAttrString(_pymsg, "comptime");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->comptime = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // input
    PyObject * field = PyObject_GetAttrString(_pymsg, "input");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 4;
      float * dest = ros_message->input;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // xref
    PyObject * field = PyObject_GetAttrString(_pymsg, "xref");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 12;
      float * dest = ros_message->xref;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // uref
    PyObject * field = PyObject_GetAttrString(_pymsg, "uref");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 4;
      float * dest = ros_message->uref;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetaz
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetaz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetaz = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetay
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetay");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetay = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetax
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetax");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetax = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dx
    PyObject * field = PyObject_GetAttrString(_pymsg, "dx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dx = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dy
    PyObject * field = PyObject_GetAttrString(_pymsg, "dy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dz
    PyObject * field = PyObject_GetAttrString(_pymsg, "dz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dz = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // omegax
    PyObject * field = PyObject_GetAttrString(_pymsg, "omegax");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->omegax = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // omegay
    PyObject * field = PyObject_GetAttrString(_pymsg, "omegay");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->omegay = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // omegaz
    PyObject * field = PyObject_GetAttrString(_pymsg, "omegaz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->omegaz = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // varphi
    PyObject * field = PyObject_GetAttrString(_pymsg, "varphi");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->varphi = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tiltvel
    PyObject * field = PyObject_GetAttrString(_pymsg, "tiltvel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tiltvel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_msgs__msg__mpc_status_old__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MPCStatusOld */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_msgs.msg._mpc_status_old");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MPCStatusOld");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  custom_msgs__msg__MPCStatusOld * ros_message = (custom_msgs__msg__MPCStatusOld *)raw_ros_message;
  {  // timestamp
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->timestamp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timestamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // status
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // comptime
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->comptime);
    {
      int rc = PyObject_SetAttrString(_pymessage, "comptime", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // input
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "input");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->input[0]);
    memcpy(dst, src, 4 * sizeof(float));
    Py_DECREF(field);
  }
  {  // xref
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "xref");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->xref[0]);
    memcpy(dst, src, 12 * sizeof(float));
    Py_DECREF(field);
  }
  {  // uref
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "uref");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->uref[0]);
    memcpy(dst, src, 4 * sizeof(float));
    Py_DECREF(field);
  }
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetaz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetaz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetaz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetay
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetay);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetay", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetax
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetax);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetax", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // omegax
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->omegax);
    {
      int rc = PyObject_SetAttrString(_pymessage, "omegax", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // omegay
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->omegay);
    {
      int rc = PyObject_SetAttrString(_pymessage, "omegay", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // omegaz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->omegaz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "omegaz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // varphi
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->varphi);
    {
      int rc = PyObject_SetAttrString(_pymessage, "varphi", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tiltvel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tiltvel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tiltvel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
