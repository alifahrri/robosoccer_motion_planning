#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

class Logger
{
public:
  Logger() {}
  std::string str()
  {
    return ss.str();
  }
  void log(std::string msg)
  {
    ss << msg;
  }
  template <typename Class>
  void log(std::string label, Class value)
  {
    ss << label << value << std::endl;
  }
  void clear()
  {
    ss.str(std::string());
  }
  void print()
  {
    std::cout << ss.str() << std::flush;
  }
  void save(std::string filename)
  {
    std::ofstream writer(filename);
    writer << ss.str();
    writer.close();
  }
  template
  <typename Class>
  Logger& operator<< (const Class &value)
  {
    ss << value;
    return *this;
  }
private:
  std::stringstream ss;
};
#endif // LOGGER_HPP
