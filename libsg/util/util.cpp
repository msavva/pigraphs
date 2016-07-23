#include "common.h"  // NOLINT

#include "util/util.h"

#include <time.h>

#include <mLibBoost.h>

#include <boost/algorithm/string.hpp>

#ifdef _WIN32
#include <windows.h>
#endif

namespace sg {
namespace util {

uint64_t timeNow() {
  FILETIME ft;
  GetSystemTimeAsFileTime(&ft);
  ULARGE_INTEGER t;
  t.HighPart = ft.dwHighDateTime;
  t.LowPart = ft.dwLowDateTime;
  return t.QuadPart / 10;
}

void decompressedGZipFile(const string& filenameIn, const string& filenameOut) {
  ifstream fileIn(filenameIn, std::ios_base::in | std::ios_base::binary);
  try {
    boost::iostreams::filtering_istream in;
    in.push(boost::iostreams::gzip_decompressor());
    in.push(fileIn);

    ofstream fileOut(filenameOut);
    for (string str; std::getline(in, str);) {
      fileOut << str << '\n';
    }
  } catch (const boost::iostreams::gzip_error& e) {
    cout << e.what() << endl;
  }
}

string timeNowMDHSstring() {
  time_t rawtime;
  tm timeinfo;
  time(&rawtime);
  localtime_s(&timeinfo, &rawtime);
  char timestr[80];
  strftime(timestr, 80, "%m-%d-%H-%M-%S", &timeinfo);
  return timestr;
}

string timeNowYMDHSstring() {
  time_t rawtime;
  tm timeinfo;
  time(&rawtime);
  localtime_s(&timeinfo, &rawtime);
  char timestr[80];
  strftime(timestr, 80, "%Y-%m-%d-%H-%M-%S", &timeinfo);
  return timestr;
}

string replaceStringAll(string subject, const string& search, const string& replace) {
  size_t pos = 0;
  while ((pos = subject.find(search, pos)) != string::npos) {
    subject.replace(pos, search.length(), replace);
    pos += replace.length();
  }
  return subject;
}

vec<string> tokenize(const string& line, const string& delimiterCharList) {
  vec<string> tokens;
  if (line.length() > 0) {
    boost::split(tokens, line, boost::is_any_of(delimiterCharList));
  }
  return tokens;
}

void tokenize(const string& line, const string& delimiterCharList, vec<string>* tokens) {
  tokens->clear();
  if (line.length() > 0) {
    boost::split(*tokens, line, boost::is_any_of(delimiterCharList));
  }
}

vec<string> tokenize(const string& line, const string& delimiterCharList, int limit, bool mergeAdjDelims) {
  vec<string> tokens;
  tokenize(line, delimiterCharList, limit, mergeAdjDelims, &tokens);
  return tokens;
}

void tokenize(const string& line, const string& delimiterCharList, int limit, bool mergeAdjDelims,
              vec<string>* tokens) {
  tokens->clear();
  if (line.length() == 0) return;
  if (limit == 1) {
    tokens->push_back(line);
    return;
  }
  const char* s = line.c_str();
  int start = 0;;
  int pos = 0;
  bool done = false;
  for (pos = 0; pos < line.length(); ++pos) {
    for (char c : delimiterCharList) {
      if (c == s[pos]) {
        // Delimiter found!!!
        if (pos > start) {
          tokens->push_back(line.substr(start, pos - start));
        } else if (!mergeAdjDelims) {
          tokens->push_back("");
        }
        done = (limit > 1 && tokens->size() == limit - 1);
        start = pos+1;
      }
      if (done) break;
    }
    if (done) break;
  }
  // Done (collect remainder)
  pos = static_cast<int>(line.length());
  if (pos > start) {
    tokens->push_back(line.substr(start, pos - start));
  }
}

vec<string> tokenizeWithQuotes(const string& line, const string& delimiterCharList, int limit, bool mergeAdjDelims) {
  vec<string> tokens;
  tokenizeWithQuotes(line, delimiterCharList, limit, mergeAdjDelims, &tokens);
  return tokens;
}

void tokenizeWithQuotes(const string& str, const string& delimiterCharList, int limit, bool mergeAdjDelims,
                        vec<string>* tokens) {
  tokens->clear();
  if (str.length() == 0) return;
  if (limit == 1) {
    tokens->push_back(str);
    return;
  }

  string buffer;
  string::const_iterator iter = str.cbegin();

  bool in_string = false;

  bool done = false;
  while (iter != str.cend() && !done) {
    char c = *iter;
    if (c == '"') {
      if (in_string) {
        tokens->push_back(buffer);
        buffer.clear();
      }
      in_string = !in_string;
    } else if (in_string) {
      buffer.push_back(c);
    } else {
      bool isDelim = false;
      for (char delim : delimiterCharList) {
        if (c == delim) {
          isDelim = true;
          break;
        }
      }
      if (isDelim) {
        if (!buffer.empty()) {
          tokens->push_back(buffer);
          buffer.clear();
        } else if (!mergeAdjDelims) {
          tokens->push_back("");
        }
        done = (limit > 1 && tokens->size() == limit - 1);
      } else {
        buffer.push_back(c);
      }
    }
    ++iter;
  }

  // Done (collect remainder)
  while (iter != str.cend()) {
    char c = *iter;
    buffer.push_back(c);
    ++iter;
  }
  if (!buffer.empty()) {
    tokens->push_back(buffer);
  }
}

string trim(const string& str) {
  string s = str;
  boost::algorithm::trim(s);
  return s;
}

void trim_in_place(string& str) {
  boost::algorithm::trim(str);
}

}  // namespace util
}  // namespace sg
