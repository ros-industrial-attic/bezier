#ifndef APPEND_BEZIER_EXCEPTION
#define APPEND_BEZIER_EXCEPTION

#include <iostream>
#include <exception>

class AppendBezierException : public std::exception
{
private:
  std::string exception_message_;

public:
  AppendBezierException(){};
  AppendBezierException(std::string exception_message)
  {
    exception_message_ = exception_message;
  }

  virtual ~AppendBezierException(){};

  virtual const char* what() const throw ()
  {
    return exception_message_.c_str();
  }
};

#endif
