#include "common.h"  // NOLINT

//#define USE_READLINE_STATIC
//#include <readline/readline.h>
//#include <readline/history.h>
#include <editline/readline.h>

#include "io/io.h"
#include "ui/CommandConsole.h"
#include "util/util.h"

namespace sg {
namespace ui {


/* Strip whitespace from the start and end of STRING.  Return a pointer
   into STRING. */
char* stripwhite (char* string) {
  register char* s, *t;

  for (s = string; isspace(*s); s++);

  if (*s == 0) { return (s); }

  t = s + strlen (s) - 1;
  while (t > s && isspace(*t)) { t--; }
  *++t = '\0';

  return s;
}

/* Execute a command line. */
int Console::execute(const string& line) const {
  // Exits the program
  if (line == "exit" || line == "quit") {
    *pTerminate_ = true;
    return 0;
  }
  // Tokenize
  const vec<string> args = sg::util::tokenizeWithQuotes(line, " ", -1, true);
  const string cmdname = args.at(0);

  // Display help
  if (cmdname == "help" || cmdname == "h") {
    if (args.size() > 1) {
      // Look up command for which help is wanted
      const string targetCmd = args.at(1);
      if (map_.count(targetCmd) > 0) {
        return map_.at(targetCmd)->showHelp(args);
      } else {
        cout << "Unknown command: " << targetCmd << endl;
      }
    } 
    // Show general help
    for (const auto& cmd : map_) {
      cout << cmd.first << " - " << cmd.second->getDescription() << endl;
    }
    cout << "exit - Exit" << endl;
    return ErrorCodes::E_OK;
  }

  // Lookup function
  if (map_.count(cmdname) > 0) {
    // Execute function
    try {
      const auto& cmd = map_.at(cmdname);
      return cmd->execute(args);
    } catch (std::exception& e) {
      cout << "An unexpected C++ error has occurred: " << e.what() << endl;
      return ErrorCodes::E_GENERAL_ERROR;
    }
  } else {
    cout << "Unknown command: " << line << endl;
    return ErrorCodes::E_OK;
  }
}

// Registers a command with the console
void Console::registerCommand(std::shared_ptr<ConsoleCommand> command) {
  const auto aliases = command->getAliases();
  for (const string& alias : aliases) {
    map_[alias] = command;
  }
}

void Console::registerExecuteCommand() {
  // Execute command
  auto execCmd = std::make_shared<sg::ui::BasicConsoleCommand>(
    "exec", "Execute commands in a file",
    [&] (const vec<string>& args) {
    if (args.size() < 2) {
      cout << "Give file of list of commands to execute" << endl;
      return sg::ui::ErrorCodes::E_INVALID_ARGS;
    }
    const string& execFile = args[1];
    vec<string> lines = io::getLines(execFile);
    for (const string& line : lines) {      
      cout << "Executing " << line << endl;
      int rv = execute(line);
      if (rv < 0) {
        cout << "Error " << rv << endl;
        return rv;
      }
    }
    
    return sg::ui::ErrorCodes::E_OK;
  });
  registerCommand(execCmd);
}

void Console::start(bool* terminate) {
  pTerminate_ = terminate;
  /* Loop reading and executing lines until the user quits. */
  char* line;
  char* s;
  while (!*pTerminate_) {
    line = readline ("> ");

    if (!line)
    { break; }

    /* Remove leading and trailing whitespace from the line.
       Then, if there is anything left, add it to the history list
       and execute it. */
    s = stripwhite (line);

    if (*s) {
      add_history(s);
      execute(s);
    }

    free (line);
  }
}

}  // namespace segmentation
}  // namespace sg
