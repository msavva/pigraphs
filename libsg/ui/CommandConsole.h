#pragma once

#include "libsg.h"  // NOLINT
#include "ui/ErrorCode.h"

#include <functional>

namespace sg {
namespace ui {

class ConsoleCommand {
 public:
  virtual const string getName() const = 0;
  virtual const string getDescription() const = 0;
  virtual const set<string> getAliases() const = 0;
  virtual int execute(const vec<string>& args) const = 0;
  virtual int showHelp(const vec<string>& args) const = 0;
};

class BasicConsoleCommand : public ConsoleCommand {
  typedef std::function<int(const vec<string>&)> ExecFn;
  public:
    BasicConsoleCommand(const string& _name, const string& _desc, const ExecFn& _execFn, const ExecFn& _showHelpFn) 
      : name_(_name)
      , desc_(_desc)
      , execFn_(_execFn)
      , showHelpFn_(_showHelpFn) { 
      aliases_.insert(_name);
    }

    BasicConsoleCommand(const string& _name, const string& _desc, const ExecFn& _execFn) 
      : name_(_name)
      , desc_(_desc)
      , execFn_(_execFn)
      , showHelpFn_([&] (const vec<string>& args) { cout << getDescription() << endl; return 0; })  { 
      aliases_.insert(_name);
    }
    const string getName() const {
      return name_;
    }
    const string getDescription() const {
      return desc_;
    }
    const set<string> getAliases() const {
      return aliases_;
    }
    int execute(const vec<string>& args) const {
      return execFn_(args);
    }
    int showHelp(const vec<string>& args) const {
      return showHelpFn_(args);
    }
    void addAlias(const string& alias) {
      aliases_.insert(alias);
    }
  private:
    const string name_;
    const string desc_;
    const ExecFn execFn_;
    const ExecFn showHelpFn_;
    std::set<string> aliases_;
};

class Console {
 public:
  void registerCommand(std::shared_ptr<ConsoleCommand> command);
  void registerExecuteCommand();
  int  execute(const string& line) const;
  void start(bool* terminate);
 private:
  //! Map of command name to command
  map<string, std::shared_ptr<ConsoleCommand>> map_;
  //! Pointer to termination bool
  bool* pTerminate_;
};

}  // namespace ui
}  // namespace sg




