#include "common.h"  // NOLINT

#include "io/json.h"

#include <rapidjson/filereadstream.h>

#include "io/io.h"

namespace sg {
namespace io {

//! Wrapper around istream for parsing with rapidjson
class istreamwrapper {
 public:
  typedef char Ch;
  istreamwrapper(istream& is) : is_(is) { }  // NOLINT
  Ch Peek() const {
    int c = is_.peek();
    return c == std::char_traits<char>::eof() ? '\0' : (Ch)c;
  }
  Ch Take() {
    int c = is_.get();
    return c == std::char_traits<char>::eof() ? '\0' : (Ch)c;
  }
  size_t Tell() const { return (size_t)is_.tellg(); }
  Ch* PutBegin() { return 0; }
  void Put(Ch) { }
  void Flush() { }
  size_t PutEnd(Ch* c) { return 0; }
 private:
  istreamwrapper(const istreamwrapper&);
  istreamwrapper& operator=(const istreamwrapper&);
  istream& is_;
};

bool parseRapidJSONDocument(const string& file, rapidjson::Document* d) {
  FILE* pFile = fopen(file.c_str(), "rb");
  char buffer[65536];
  rapidjson::FileReadStream is(pFile, buffer, sizeof(buffer));
  d->ParseStream<0, rapidjson::UTF8<>, rapidjson::FileReadStream>(is);
  fclose(pFile);

  if (d->HasParseError()) {
    cerr << "Parse error reading " << file << endl
      << "Error code " << d->GetParseError() << " at " << d->GetErrorOffset() << endl;
    return false;
  } else {
    return true;
  }
}

bool parseRapidJSONString(const string& jsonString, rapidjson::Document* d) {
  d->Parse(jsonString.c_str());

  if (d->HasParseError()) {
    cerr << "Parse error parsing json string " << endl
      << "Error code " << d->GetParseError() << " at " << d->GetErrorOffset() << endl;
    return false;
  } else {
    return true;
  }
}

}  // namespace io
}  // namespace sg
