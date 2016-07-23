#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "ui/ErrorCode.h"
#include "util/util.h"

namespace sg {
namespace ui {

template <typename Signal = string>
class EventMap {
 public:
  typedef vec<Signal> Tokens;
  typedef std::function<Tokens(const Signal&)> Tokenizer;
  typedef std::function<ErrorCode(const Tokens&)> ActionType;

  EventMap() : map_(), tokenizer_([](const Signal& s) { return sg::util::tokenizeWithQuotes(s, " ", -1, true); }) { }

  explicit EventMap(const vec<pair<Signal, ActionType>>& events, const Tokenizer& _tokenizer)
    : map_(events), tokenizer_(_tokenizer) { }

  inline void init(const std::unordered_map<Signal, ActionType>& _map,
  const Tokenizer& _tokenizer = [](const string& s) { return sg::util::tokenizeWithQuotes(s, " ", -1, true); }) {
    map_ = _map;
    tokenizer_ = _tokenizer;
  }

  inline void processSignalQueue(vec<Signal>* signals, std::recursive_mutex* mutex) const {
    mutex->lock();
    for (const Signal& s : *signals) { processTokens(tokenizer_(s)); }
    signals->clear();
    mutex->unlock();
  }

  inline int processTokens(const Tokens& tokens) const {
    try {
      const Signal& command = tokens[0];
      if (map_.find(command) == map_.end()) {
        cout << "Unknown signal command: " << command << endl;
        return ErrorCodes::E_UNKNOWN_CMD;
      } else {
        int rv = map_.at(command)(tokens);
        if (rv < 0) {
          cout << "Error processing command: " << rv << endl;
        }
        return rv;
      }
    } catch (std::exception& e) {
      cout << "An unexpected C++ error has occurred: " << e.what() << endl;
      return ErrorCodes::E_GENERAL_ERROR;
    }
  }

  inline ActionType& operator[] (const Signal& s) { return map_[s]; }

  inline void getSignals(vec<string>* pKeys) {
    pKeys->clear();
    for (const auto& it : map_) {
      pKeys->push_back(it.first);
    }
  }

  inline void disconnect(const Signal& s) { map_.erase(s); }

 private:
  std::unordered_map<Signal, ActionType> map_;
  Tokenizer tokenizer_;
};

}  // namespace ui
}  // namespace sg


