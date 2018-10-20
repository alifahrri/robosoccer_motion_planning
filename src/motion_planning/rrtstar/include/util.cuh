#ifndef DEVICE_UTIL_H
#define DEVICE_UTIL_H

#include <cuda.h>
#include <cuda_runtime.h>

#define HOST __host__
#define DEVICE __device__
#define ATTRIBUTE __host__ __device__

#ifndef gpuErrchk
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
  if (code != cudaSuccess)
  {
    fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) exit(code);
  }
}
#endif

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

template <typename ObjectType>
struct DevAllocator
{
  DevAllocator(size_t n)
  {
    allocate(n);
  }

  ~DevAllocator()
  {
    gpuErrchk(cudaFree(ptr));
  }

  inline
  void allocate(size_t n_obj)
  {
    n = n_obj;
    n_bytes = (n*sizeof(ObjectType));
    gpuErrchk(cudaMalloc(&ptr, n_bytes));
  }

  inline
  bool ok(size_t size)
  {
    return n > size;
  }

  inline
  void resize(size_t new_size)
  {
    gpuErrchk(cudaFree(ptr));
    allocate(new_size);
  }

  inline
  void copy_from(const ObjectType *host, size_t n)
  {
    gpuErrchk(cudaMemcpy(ptr, host, n*sizeof(ObjectType), cudaMemcpyHostToDevice));
  }

  inline
  void copy_to(ObjectType *host, size_t n)
  {
    gpuErrchk(cudaMemcpy(host, ptr, n*sizeof(ObjectType), cudaMemcpyDeviceToHost));
  }

  ObjectType *ptr;
  size_t n;
  size_t n_bytes;
};

template <typename ObjectType>
struct HostAllocator
{
  HostAllocator(size_t n)
  {
    allocate(n);
  }

  ~HostAllocator()
  {
    // delete[] ptr;
    cudaFreeHost(ptr);
  }

  inline
  void allocate(size_t n_obj)
  {
    n = n_obj;
    n_bytes = (n*sizeof(ObjectType));
    // ptr = new ObjectType[n];
    // use pinned mem for faster cudaMemcpy
    cudaMallocHost(&ptr, n_bytes);
  }

  inline
  bool ok(size_t size)
  {
    return n > size;
  }

  inline
  void resize(size_t new_size)
  {
    // delete[] ptr;
    cudaFreeHost(ptr);
    allocate(new_size);
  }

  ObjectType *ptr;
  size_t n;
  size_t n_bytes;
};

#endif // DEVICE_UTIL_H
