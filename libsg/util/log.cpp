#include "common.h"  // NOLINT

#include "util/log.h"
#include "util/util.h"

#include <boost/phoenix/bind.hpp>
#include <boost/regex.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/expressions/predicates/channel_severity_filter.hpp>
//#include <boost/log/sinks/async_frontend.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/value_ref.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/from_stream.hpp>
//#include <boost/log/sources/severity_logger.hpp>
//#include <boost/log/sources/record_ostream.hpp>


namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace sinks = boost::log::sinks;
namespace attrs = boost::log::attributes;
namespace expr = boost::log::expressions;

typedef sinks::synchronous_sink<sinks::text_file_backend> sink_t;

using boost::log::trivial::severity_level;

BOOST_LOG_ATTRIBUTE_KEYWORD(line_id, "LineID", unsigned int)
BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", severity_level)
BOOST_LOG_ATTRIBUTE_KEYWORD(channel, "Channel", std::string)
BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", std::string)
BOOST_LOG_ATTRIBUTE_KEYWORD(scope, "Scope", std::string)

namespace sg {
namespace util {

string defaultLogFormat = "%TimeStamp% [%Severity%] [%Scope%]: %Channel% %Message%";
map<string, boost::shared_ptr<sink_t>> sinks;

// See http://www.boost.org/doc/libs/1_60_0/libs/log/doc/html/log/detailed/expressions.html
typedef expr::channel_severity_filter_actor< std::string, severity_level > min_severity_filter;
min_severity_filter min_severity = expr::channel_severity_filter(channel, severity);

map<string, severity_level> severityLevelMap;

//! Initializes logging from a settings file
//! See http://www.boost.org/doc/libs/1_57_0/libs/log/doc/html/log/detailed/utilities.html#log.detailed.utilities.setup.settings_file
void initLogging(const string& settingsFilename) {
  cout << "Getting log settings from " << settingsFilename << endl;
  ifstream file(settingsFilename);
  logging::init_from_stream(file);
}

// See http://gernotklingler.com/blog/simple-customized-logger-based-boost-log-v2/
void initLogging(Params& params) {
  severityLevelMap["trace"] = boost::log::trivial::trace;
  severityLevelMap["debug"] = boost::log::trivial::debug;
  severityLevelMap["info"] = boost::log::trivial::info;
  severityLevelMap["warning"] = boost::log::trivial::warning;
  severityLevelMap["error"] = boost::log::trivial::error;
  severityLevelMap["fatal"] = boost::log::trivial::fatal;

  logging::add_common_attributes();
  logging::register_simple_formatter_factory< severity_level, char >("Severity");
  logging::core::get()->add_global_attribute("Scope", attrs::named_scope());

  if (params.exists("logSettingsFile")) {
    // Use fancy log settings file
    initLogging(params.get<string>("logSettingsFile"));
  } else if (params.exists("logFile")) {
    const string& logFilename = params.get<string>("logDir") + params.get<std::string>("logFile");
    add_file_log(logFilename);
    
    const auto console_sink = logging::add_console_log(
      std::clog,
      keywords::format = "[%Severity%] [%Scope%]: %Channel% %Message%"
    );
    //sinks["console"] = console_sink;

    // Overall severity
    const string severityLevelStr = params.getWithDefault<string>("logSeverity", "info");
    severity_level severityLevel = severityLevelMap.at(severityLevelStr);

    // List of components we plan to debug
    const string logDebugStr = params.getWithDefault<string>("logDebug", "");
    vec<string> debugVec = util::tokenize(logDebugStr, ",");
    set<string> debugSet(debugVec.begin(), debugVec.end());
    for (const string& v : debugVec) {
      min_severity[v] = boost::log::trivial::debug;
    }

    // See http://boost-log.sourceforge.net/libs/log/doc/html/log/tutorial/advanced_filtering.html
    logging::core::get()->set_filter(
      logging::trivial::severity >= severityLevel || min_severity 
    );
  }
}

void set_log_severity(const string& channel, const string& levelStr) {
  if (severityLevelMap.count(levelStr)) {
    severity_level level = severityLevelMap.at(levelStr);
    min_severity[channel] = level;
  }
}

void set_log_severity(const string& channel, const severity_level level) {
  min_severity[channel] = level;
}

void add_file_log(const string& filename, bool autoFlush) {
  if (sinks.count("file:" + filename) == 0) {
    const auto sink = logging::add_file_log(
      keywords::file_name = filename,
      keywords::format = defaultLogFormat
    );
    if (autoFlush) {
      sink->locked_backend()->auto_flush(true);
    }
    sinks["file:" + filename] = sink;
  }
}

void stop_logging(boost::shared_ptr<sink_t>& sink) {
  boost::shared_ptr<logging::core> core = logging::core::get();

  // Remove the sink from the core, so that no records are passed to it
  core->remove_sink(sink);

  // Break the feeding loop
  //sink->stop();

  // Flush all log records that may have left buffered
  sink->flush();

  sink.reset();
}

void stop_logging() {
  for (auto& it : sinks) {
    stop_logging(it.second);
  }
  sinks.clear();
}

void remove_file_log(const string& filename) {
  const auto& it = sinks.find("file:" + filename);
  if (it != sinks.end()) {
    stop_logging(it->second);
    sinks.erase(it);
  }
}

void flush_logs() {
  for (auto& sink : sinks) {
    sink.second->flush();
  }
}

}  // namespace util
}  // namespace sg
