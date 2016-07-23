#include "common.h"  // NOLINT

#include "util/timer.h"

#include <boost/timer/timer.hpp>

namespace sg {
namespace util {

Timer::Timer(const string& msg, ostream& os /*= std::cout*/)
  : m_msg(msg)
  , m_os(os)
  , m_timer(new boost::timer::auto_cpu_timer(m_os, 3, "%ws\n")) { }

Timer::~Timer() {
  m_os << m_msg << " : ";
  if (m_timer != nullptr) { delete m_timer; }
}

}  // namespace util
}  // namespace sg
