#pragma once

#include <boost/log/trivial.hpp>
#include <boost/log/common.hpp>

namespace sg {
namespace util {

// We use the default log levels (trace, debug, info, warning, error, fatal)
#define SG_LOG  BOOST_LOG_TRIVIAL

#define SG_LOG_FUNC  BOOST_LOG_FUNCTION
#define SG_LOG_SCOPE BOOST_LOG_NAMED_SCOPE
#define SG_LOG_TAG   BOOST_LOG_SCOPED_THREAD_TAG

#define SG_LOG_TRACE BOOST_LOG_TRIVIAL(trace)
#define SG_LOG_DEBUG BOOST_LOG_TRIVIAL(debug)
#define SG_LOG_INFO  BOOST_LOG_TRIVIAL(info)
#define SG_LOG_WARN  BOOST_LOG_TRIVIAL(warning)
#define SG_LOG_ERROR BOOST_LOG_TRIVIAL(error) << __FILE__ << ":" << __LINE__ << "[" << __FUNCTION__ << "] "
#define SG_LOG_FATAL BOOST_LOG_TRIVIAL(fatal) << __FILE__ << ":" << __LINE__ << "[" << __FUNCTION__ << "] "

#define SG_DEBUG_MSG(s) (std::string(__FILE__) + ":" + std::to_string(__LINE__) + "[" + std::string(__FUNCTION__) + "]: " + (s))

#define SG_EXCEPTION(s) std::runtime_error(SG_DEBUG_MSG(s))

//! Initializes logging from a settings file
//! See http://www.boost.org/doc/libs/1_57_0/libs/log/doc/html/log/detailed/utilities.html#log.detailed.utilities.setup.settings_file
void initLogging(const std::string& settingsFilename);

//! Initializes logging from params
//! Logs if file if following params are set: logFile, logDir
void initLogging(sg::util::Params& params);

//! set log severity
void set_log_severity(const string& channel, const string& level);
void set_log_severity(const string& channel,
                      const boost::log::trivial::severity_level level);

//! Add logging to given filename
void add_file_log(const string& filename, bool autoFlush = false);
//! Remove logging to given filename
void remove_file_log(const string& filename);

//! Flush state
void flush_logs();

//! Stop all logging
void stop_logging();

}  // namespace util
}  // namespace sg


