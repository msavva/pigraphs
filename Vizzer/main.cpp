#include "common.h"  // NOLINT

#include <string>
#include <vector>

#include <core/Database.h>
#include <io/binvox.h>
#include <net/net.h>
#include <ui/CommandConsole.h>
#include <util/Params.h>
#include <util/util.h>
#include "../jace-proxy/nlputil.h"
#include "../jace-proxy/wekautil.h"

#include "./SharedAppState.h"
#include "./D3D11Vizzer.h"
#include "util/log.h"

using std::string;  using std::cout;  using std::cerr;  using std::endl;  using std::vector;

void launchD3D11Vizzer(SharedAppState& state, sg::util::Params* params) {  // NOLINT
  ml::Console::openLogFile("console.txt");
  D3D11Vizzer callback(state, params);
  ml::ApplicationWin32 app(nullptr, 640, 640, "D3D11Vizzer", ml::GraphicsDeviceTypeD3D11, callback);
  app.messageLoop();
}

// Boots up Vizzer UI with given parameters p and SharedAppState state
int startVizzer(sg::util::Params& p, SharedAppState& state) {  // NOLINT
  if (state.isVizStarted) {
    cout << "Vizzer already started!" << endl;
    return 0;
  }

  // Initialize JVM
  if (wekautil::initJVM() != 0) {
    cerr << "Failure initializing JVM through jace. Check that jvm.dll is in PATH." << endl;
  }

  // Check that dataDir is accessible and exit after reporting error if it is not
  const string dataDir = p.get<string>("dataDir");
  if (!boost::filesystem::is_directory(dataDir)) {
    cerr << "[ERROR] dataDir=" << dataDir << " is inaccessible. Check connection or parameters file." << endl;
    //int c = getchar();
    //exit(-1);
  }

  // Init db and UI
  state.database = new sg::core::Database(p);
  state.ui.init("../UIWindow.exe", "UnderstandingScenes");

  state.vizzerThread = std::thread(launchD3D11Vizzer, std::ref(state), &p);
  state.isVizStarted = true;

  return 0;
}

sg::ui::Console console;

void launchConsole(SharedAppState& state, sg::util::Params* params) {  // NOLINT
  // Start command console
  console.registerExecuteCommand();

  // Parse command
  auto parseActionsCmd = std::make_shared<sg::ui::BasicConsoleCommand>(
    "parse", "Parses text into actions", 
    [](const vector<string>& args) {
      vector<string> rest = args;
      rest.erase(rest.begin());
      string text = sg::util::join(rest, " ");
      vector<nlputil::Action> actions;
      int rv = nlputil::parseActions(text, &actions);
      if (rv >= 0) {
        for (const nlputil::Action& action : actions) {
          cout << action << endl;
        }
      } else {
        cout << "ERROR: " << rv << endl;
      }
      return rv;
   });
  console.registerCommand(parseActionsCmd);

  // Dump PLY command
  auto dumpRecordingPLYCmd = std::make_shared<sg::ui::BasicConsoleCommand>(
    "dumpPLY", "Reads recording depth frames (.depth.avi) and dumps .sensor and .ply file",
    [ ] (const vector<string>& args) {
    if (args.size() < 2) {
      cout << "Give path to .depth.avi file as argument" << endl;
      return sg::ui::ErrorCodes::E_INVALID_ARGS;
    }
    const int numFrames = (args.size() > 2) ? std::stoi(args[2]) : 10;
    const string& depthFile = args[1];
    vizutil::dumpDepthFramesToSensorFileAndPLY(depthFile, numFrames);
    return sg::ui::ErrorCodes::E_OK;
  });
  console.registerCommand(dumpRecordingPLYCmd);

  // Combine voxels command
  auto combineVoxelsCmd = std::make_shared<sg::ui::BasicConsoleCommand>(
    "combineVoxels", "Reads surface and solid voxels and combines them",
    [] (const vector<string>& args) {
    if (args.size() == 2) {
      // Assume batch processing
      cout << "Batch merge from " << args[1];
      sg::io::merge_binvox_batch(args[1]);
      return sg::ui::ErrorCodes::E_OK;
    }

    if (args.size() < 4) {
      cout << "Give input1,input2, and output files as argument" << endl;
      return sg::ui::ErrorCodes::E_INVALID_ARGS;
    }
    int ok = 0;
    if (sg::io::isDirectory(args[1])) {
      if (args.size() > 4) {
        ok = sg::io::merge_binvox_dir(args[1], args[2], args[3], args[4]);
      } else {
        ok = sg::io::merge_binvox_dir(args[1], args[2], args[3]);        
      }
    } else {
      if (sg::io::fileExists(args[3])) {
        cout << "Output file " << args[3] << " already exists" << endl;
        return sg::ui::ErrorCodes::E_GENERAL_ERROR;
      }
      ok = sg::io::merge_binvox(args[1], args[2], args[3]);
    }

    if (ok) {
      return sg::ui::ErrorCodes::E_OK;
    } else {
      return sg::ui::ErrorCodes::E_GENERAL_ERROR;
    }      
  });
  console.registerCommand(combineVoxelsCmd);

  // convertRecToJson command
  auto convertRecToJsonCmd = std::make_shared<sg::ui::BasicConsoleCommand>(
    "convertRecToJSON", "Batch convert .rec format recordings in a directory to .json format",
    [ ] (const vector<string>& args) {
    if (args.size() < 2) {
      cout << "Give path to directory with .rec files as argument" << endl;
      return sg::ui::ErrorCodes::E_INVALID_ARGS;
    }
    const string& recDir = args[1];
    sg::core::RecordingDatabase::batchConvertRecToJSON(recDir);
    return sg::ui::ErrorCodes::E_OK;
  });
  console.registerCommand(convertRecToJsonCmd);

  // startViz command
  auto startVizCmd = std::make_shared<sg::ui::BasicConsoleCommand>(
    "start", "Start Vizzer UI",
    [&] (const vector<string>& args) {
      startVizzer(*params, state);
    return sg::ui::ErrorCodes::E_OK;
  });
  console.registerCommand(startVizCmd);

  // Flush logs
  auto flushLogsCmd = std::make_shared<sg::ui::BasicConsoleCommand>(
    "flush", "Flush logs",
    [&] (const vector<string>& args) {
    sg::util::flush_logs();
    return sg::ui::ErrorCodes::E_OK;
  });
  console.registerCommand(flushLogsCmd);

  // Forward commands to Vizzer
  auto vizCmd = std::make_shared<sg::ui::BasicConsoleCommand>(
    "viz",
    "Issues Vizzer command", 
    [&] (const vector<string>& args) {
      vector<string> rest = args;
      rest.erase(rest.begin());
      const string text = "D3D:" + sg::util::join(rest, " ");
      state.sendMessage(SharedAppState::UI_D3D, text);
      return sg::ui::ErrorCodes::E_OK;
    },
    [&] (const vector<string>& args) {
      vector<string> rest = args;
      rest.erase(rest.begin());
      rest[0] = "help";
      const string text = "D3D:" + sg::util::join(rest, " ");
      state.sendMessage(SharedAppState::UI_D3D, text);
      return sg::ui::ErrorCodes::E_OK;
    }
  );
  vizCmd->addAlias("v");
  console.registerCommand(vizCmd);

  cout << "Welcome to SceneGrok! Type help for usage info." << endl;
  // start our console!
  console.start(&state.terminate);
}

void launchHttpServer(SharedAppState& state, sg::net::HTTPServer** ppServer) {  // NOLINT
  using sg::net::Request;  using sg::net::Response;
  const string address = "localhost";
  const string port = "8888";
  sg::net::HTTPServer* pServer = new sg::net::HTTPServer(address, port);
  *ppServer = pServer;
  sg::net::HTTPServer::HandlerFun hello = [] (const Request& req, Response* pRes) {
    pRes->contentType = "text/plain";
    pRes->body = "Hello there! You came for: " + req.destination;
  };
  pServer->addHandler("/", hello);
  pServer->addHandler("/hello", hello);
  pServer->addHandler("/favicon.ico", [] (const Request& req, Response* pRes) {
    pRes->contentType = "image/gif";
    pRes->body = "data:image/gif;base64,R0lGODlhAQABAIAAAP///wAAACwAAAAAAQABAAACAkQBADs=";
  });

  SG_LOG_INFO << "Starting server at " << address << ":" << port;
  pServer->run();
}


void positionConsoleWindow() {
  system("mode CON: COLS=180");  // Set console window width
  HWND hWnd = GetConsoleWindow();
  HMONITOR hMonitor = MonitorFromWindow(hWnd, MONITOR_DEFAULTTOPRIMARY);
  MONITORINFO mi;
  mi.cbSize = sizeof(mi);
  GetMonitorInfo(hMonitor, &mi);
  const int w = 850, h = 350;
  const int x = mi.rcWork.right - w;
  const int y = mi.rcWork.top + 60;
  MoveWindow(hWnd, x, y, w, h, true);
}

int main(int argc, const char** argv) {
  SG_LOG_SCOPE("main");
  positionConsoleWindow();

  // Create parameter and state structs
  SharedAppState state;
  sg::util::Params p;
  const string
    defaultParamsFile = "../parameters_default.txt",
    paramsFile        = "../parameters.txt";
  if (sg::io::fileExists(defaultParamsFile)) {
    p.read(defaultParamsFile);
  } else {
    cerr << "WARNING: Default parameters file not found at " << defaultParamsFile << ". Expect errors." << endl;
    p.set<bool>("autostartVizzer", false);  // don't try to start up Vizzer UI since we don't have params
  }
  p.updateIfExists(paramsFile);
  sg::util::initLogging(p);

  // Start console thread
  std::thread consoleThread(launchConsole, std::ref(state), &p);
  // Start http server thread
  sg::net::HTTPServer* pHTTPServer = nullptr;
  std::thread httpServerThread(launchHttpServer, std::ref(state), &pHTTPServer);

  // autostart Vizzer UI
  if (p.get<bool>("autostartVizzer")) {
    startVizzer(p, state);
  }

  // Join threads and exit
  SG_LOG_INFO << "Waiting...";

  // flush logs in case there is hanging later
  sg::util::flush_logs();

  SG_LOG_INFO << "Waiting for console thread...";
  if (consoleThread.joinable()) {
    consoleThread.join();
  }

  SG_LOG_INFO << "Waiting for http thread...";
  pHTTPServer->stop();
  if (httpServerThread.joinable()) {
    httpServerThread.join();
  }
  delete pHTTPServer;

  SG_LOG_INFO << "Waiting for Vizzer thread...";
  if (state.isVizStarted && state.vizzerThread.joinable()) {
    state.vizzerThread.join();
  }

  SG_LOG_INFO << "Done. Goodbye!";
  sg::util::stop_logging();

  return 0;
}
