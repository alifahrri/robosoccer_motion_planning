#ifndef VECTORWRAPPER_HPP
#define VECTORWRAPPER_HPP

#include <iostream>
#include <vector>
#include <memory>

#include "boost/shared_ptr.hpp"
#include "boost/python.hpp"
#include "boost/python/stl_iterator.hpp"

using namespace std;

template<typename T>
inline
std::vector<T> py_list_to_std_vector(const boost::python::object &iterable)
{
    std::vector<T> ret;
    boost::python::stl_input_iterator<T> begin(iterable), end();
    auto n = boost::python::len(iterable);
    for(size_t i=0; i<n; i++) {
      ret.push_back(boost::python::extract<T>(iterable[i]));
    }
    return ret;
}

template <class T>
inline
boost::python::list std_vector_to_py_list(std::vector<T> vector) {
  typename std::vector<T>::iterator iter;
  boost::python::list list;
  for (iter = vector.begin(); iter != vector.end(); ++iter) {
    list.append(*iter);
  }
  return list;
}

#endif // VECTORWRAPPER_HPP
