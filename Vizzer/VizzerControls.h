#pragma once

#include <mLibCore.h>
#include <mLibD3D11.h>

class D3D11Vizzer;
namespace ml { struct ApplicationData; }

class VizzerControls {
 public:
  virtual ~VizzerControls() { }

  explicit VizzerControls(D3D11Vizzer* pVizzer);
  virtual void keyDown(ml::ApplicationData& app, unsigned int key);  // NOLINT
  virtual void keyPressed(ml::ApplicationData& app, unsigned int key);  // NOLINT
  virtual void mouseDown(ml::ApplicationData& app, ml::MouseButtonType button);  // NOLINT
  virtual void mouseMove(ml::ApplicationData& app);
  virtual void mouseWheel(ml::ApplicationData& app, int wheelDelta);

 private:
  D3D11Vizzer* m_viz;
};

