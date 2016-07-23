#pragma once

#include "libsg.h"  // NOLINT

#include <functional>

// Forward declaration
namespace boost { namespace timer { class auto_cpu_timer; } }

namespace sg {
namespace util {

//! Simple Timer class that reports a message along with elapsed wall clock time when desctructed
class Timer {
 public:
  explicit Timer(const string& _msg, ostream& os = std::cout);
  ~Timer();

 private:
  const string m_msg;
  ostream& m_os;
  boost::timer::auto_cpu_timer* m_timer;
};

}  // namespace util
}  // namespace sg


