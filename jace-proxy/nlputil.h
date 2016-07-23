#pragma once

#include <string>
#include <vector>

namespace nlputil {

  struct Action {
    std::string agent;
    std::string verb;
    std::string object;
  };
  inline std::ostream& operator<<(std::ostream& os, const Action& a) {  // NOLINT
    os << a.agent << "-" << a.verb << "-" << a.object;
    return os;
  }

  //! Initializes CoreNLP (needs to be called before any other interface operations
  int initNLP();

  //! Populates pVectors with actions parsed from the given text
  //! Returns >=0 if successful
  int parseActions(const std::string text, std::vector<Action>* pActions);

}  // namespace nlputil


