#include "./nlputil.h"
#include "./wekautil.h"

#include "jace/proxy/java/lang/String.h"
#include "jace/proxy/java/util/List.h"
#include "jace/proxy/java/util/ArrayList.h"
#include "jace/proxy/edu/stanford/graphics/parser/Action.h"
#include "jace/proxy/edu/stanford/graphics/parser/ActionParser.h"
#include "jace/proxy/edu/stanford/graphics/parser/Mention.h"

#include "jace/Jace.h"
#include "jace/JArray.h"

namespace nlputil {
  typedef jace::proxy::java::lang::String JString;
  typedef jace::proxy::java::util::List JList;
  typedef jace::proxy::edu::stanford::graphics::parser::Action JAction;
  typedef jace::proxy::edu::stanford::graphics::parser::ActionParser JActionParser;
  typedef jace::proxy::edu::stanford::graphics::parser::Mention JMention;

  static bool nlpInited = false;
  static JActionParser* pActionParser = new JActionParser();

  int initNLP() {
    if (!nlpInited) {
      int jvmStatus = wekautil::initJVM();
      if (jvmStatus < 0) {
        return jvmStatus;
      }
      try {
        JNIEnv* env = jace::attach();
        *pActionParser = jace::java_new<JActionParser>();
        nlpInited = true;
        return 0;
      } catch (...) {
        return wekautil::handleJaceExceptions();
      }
    } else {
      return 0;
    }
  }

  int parseActions(const std::string text, std::vector<Action>* pActions) {
    int res = initNLP();
    if (res >= 0) {
      try {
        JNIEnv* env = jace::attach();
        auto jactions = pActionParser->parse(text);
        int size = jactions.size();
        pActions->clear();
        pActions->reserve(size);
        for (int i = 0; i < size; ++i) {
          JAction action = jace::java_cast<JAction>(jactions.get(i));
          std::string agent = action.getAgent().getText();
          std::string verb = action.getVerb().getText();
          std::string object = action.getObject().getText();
          pActions->emplace_back(Action{agent, verb, object});
        }
        res = 0;
      } catch (...) {
        res = wekautil::handleJaceExceptions();
      }
    }
    return res;
  }
}
