#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <cstdio>

#define DEBUG_PRINT(VAR, VAL) { print(VAR, VAL); }
#define TRACE_FN(NAME) { print_fn(NAME); }
inline void print_fn(const char *fn_name)
{
  printf("%s: \n", fn_name);
}
template <typename Type>
inline void print(const char *var, Type val)
{
  std::cout << var << ": " << val << std::endl;
}

#endif // UTIL_H
