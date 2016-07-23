#include "common.h"  // NOLINT

#include "net/net.h"

// define below stops boost from complaining about undefed win version
#define _WIN32_WINNT 0x0602
#pragma warning(disable:4267)
#include <boost/network/protocol/http/client.hpp>
#include <boost/network/protocol/http/server.hpp>
#pragma warning(default:4267)

using namespace boost::network;

namespace sg {
namespace net {

string get(const string& url) {
  http::client client;
  try {
    http::client::request request(url);
    http::client::response response = client.get(request);
    return static_cast<string>(body(response));
  } catch (std::exception& e) {
    SG_LOG_ERROR << "HTTP GET errored: " << e.what();
    return "";
  }
}

bool get(const string& url, const string& path) {
  const string res = get(url);
  if (res.size()) {
    ofstream ofs(path);
    ofs << res;
    ofs.close();
    return true;
  }
  return false;
}

string query(const string& url, const map<string, string>& params) {
  uri::uri baseUrl(url);
  uri::uri queryUrl;
  queryUrl << baseUrl;
  for (const auto& p : params) {
    queryUrl << uri::query(uri::encoded(p.first), uri::encoded(p.second));
  }
  SG_LOG_INFO << "Querying: " << queryUrl.string();
  return get(queryUrl.string());
}

typedef http::server<HTTPServer::Dispatcher> server;
struct HTTPServer::Dispatcher {
  void operator() (server::request const& request, server::response& response) const {

    const string dest = request.destination;
    if (m_handlers.count(dest) > 0) {
      const HandlerFun& handler = m_handlers.at(dest);
      Request req{source(request), dest, method(request), body(request)};
      Response res;
      try {
        handler(req, &res);
        response = server::response::stock_reply(server::response::ok, res.body);
        response.headers[1].value = res.contentType;
      } catch (std::exception& e) {
        response = server::response::stock_reply(kErr, "Something went wrong!");
        SG_LOG_ERROR << "Server error: " << e.what();
      }
    } else {
      response = server::response::stock_reply(kErr, "No handler for dest: " + dest);
      SG_LOG_ERROR << "No handler for " << dest;
    }
  }

  void log(...) { }

  void addHandler(const string& attachmentPoint, const HandlerFun& handler) {
    m_handlers[attachmentPoint] = handler;
  }

 private:
  static const auto kErr = server::response::internal_server_error;
  map<string, HandlerFun> m_handlers;
};

HTTPServer::HTTPServer(const string& address, const string& port) {
  m_pDispatcher = new Dispatcher();
  server::options options(*m_pDispatcher);
  m_pServer = new server(options.address(address).port(port));
}

HTTPServer::~HTTPServer() {
  if (m_pServer) {
    delete static_cast<server*>(m_pServer);
  }
  if (m_pDispatcher) {
    delete m_pDispatcher;
  }
}

void HTTPServer::addHandler(const string& attachmentPoint, const HandlerFun& handler) {
  m_pDispatcher->addHandler(attachmentPoint, handler);
}

void HTTPServer::run() {
  if (m_pServer) {
    m_isRunning = true;
    try {
      static_cast<server*>(m_pServer)->run();
    } catch (std::exception& e) {
      m_isRunning = false;
      SG_LOG_ERROR << "Error starting server: " << e.what();
    }
  }
}

void HTTPServer::stop() {
  if (m_pServer) {
    m_isRunning = false;
    static_cast<server*>(m_pServer)->stop();
  }
}

}  // namespace net
}  // namespace sg
