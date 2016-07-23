#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace net {

//! HTTP get from given url and return response as string
string get(const string& url);

//! HTTP get a file from the given url and save it to the given local path
//! Return whether succeded
bool get(const string& url, const string& path);

//! Encodes query params in query and requests from endpoint at url
//! Returns response as string
string query(const string& url, const map<string, string>& params);

//! HTTP request
struct Request {
  string source;
  string destination;
  string method;
  string body;
};

//! HTTP response
struct Response {
  string contentType;
  string body;
};

//! A simple HTTP server class
class HTTPServer {
 public:
  HTTPServer(const string& address, const string& port);
  ~HTTPServer();

  //! HTTP Server handler function
  typedef std::function<void(const Request& req, Response* pRes)> HandlerFun;
  void addHandler(const string& attachmentPoint, const HandlerFun& handler);

  struct Dispatcher;

  void run();
  void stop();

 private:
  bool m_isRunning = false;
  Dispatcher* m_pDispatcher;
  void* m_pServer;  // Ugly, but need to store http::server<Dispatcher> ptr without including here
};

}  // namespace net
}  // namespace sg


