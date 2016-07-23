#pragma once

#include <string>

#include <ui/EventMap.h>

class D3D11Vizzer;
namespace ml { struct ApplicationData; }

class VizzerEventMap {
 public:
  void populate(D3D11Vizzer* pVizzer, ml::ApplicationData& app);  // NOLINT
  void processSignalQueue();

 private:
  sg::ui::EventMap<std::string> m_eventMap;
  D3D11Vizzer* m_viz;
};


