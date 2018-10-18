#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <cstdio>
#ifdef __NVCC__
#include <cuda.h>
#include <cuda_runtime.h>
#define TRACE_KERNEL_STR1(ID, DEBUG_ID, STR) { if(ID == DEBUG_ID) printf("[%d] %s\n",ID,STR); }
#define TRACE_KERNEL_STR2(ID, DEBUG_ID, STR1, STR2) { if(ID == DEBUG_ID) {printf("[%d] %s: %s\n",ID,STR1,STR2);} }
#define ATTRIBUTE __host__ __device__
#else
#define ATTRIBUTE
#endif

#define DEBUG_PRINT(VAR, VAL) { print(VAR, VAL); }
#define TRACE_FN(NAME) { print_fn(NAME); }

ATTRIBUTE
inline void print(const char *str)
{
  printf("%s\n", str);
}
ATTRIBUTE
inline void print_fn(const char *fn_name)
{
  printf("%s: \n", fn_name);
}
template <typename Type>
inline void print(const char *var, Type val)
{
  std::cout << var << ": " << val << std::endl;
}

#ifdef __NVCC__
#define GET_MACRO(_1,_2,_3,_4,NAME,...) NAME
#define TRACE_KERNEL(...) GET_MACRO(__VA_ARGS__, TRACE_KERNEL_STR2, TRACE_KERNEL_STR1)(__VA_ARGS__)
#endif

#endif // UTIL_H
