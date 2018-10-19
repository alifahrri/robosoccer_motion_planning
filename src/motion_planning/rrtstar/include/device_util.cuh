#ifndef DEVICE_UTIL_H
#define DEVICE_UTIL_H

#include <cuda.h>
#include <cuda_runtime.h>

#define HOST __host__
#define DEVICE __device__
#define ATTRIBUTE __host__ __device__

// range based for loop wrapper for generic object
template <typename ObjectType>
struct RangeWrapper
{
  __host__ __device__
  RangeWrapper(ObjectType *obj, size_t n)
    : obj(obj), n(n), should_free(false)
  {}

  __host__ __device__
  RangeWrapper(size_t n)
    : n(n), should_free(true)
  {
    obj = (ObjectType*)malloc(n * sizeof(ObjectType));
  }

  __host__ __device__
  ~RangeWrapper()
  {
    if(should_free)
      free(obj);
  }

  __host__ __device__
  size_t size() const { return n; }

  __host__ __device__
  ObjectType& operator[](size_t i) {
    return obj[i];
  }

  __host__ __device__
  const ObjectType& operator[](size_t i) const {
    return obj[i];
  }

  __host__ __device__
  ObjectType* begin() {
    return (obj[0]);
  }

  __host__ __device__
  const ObjectType* begin() const {
    return &(obj[0]);
  }

  __host__ __device__
  ObjectType* end() {
    return &(obj[n]);
  }

  __host__ __device__
  const ObjectType* end() const {
    return &(obj[n]);
  }

  ObjectType *obj;
  size_t n;
  bool should_free;
};

#endif // DEVICE_UTIL_H
