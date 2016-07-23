#pragma once

#include "libsg.h"  // NOLINT

#include "util/index.h"

namespace sg {
namespace vis {

class ColorIndex {
 public:
  static string rgbColorToHex(const ml::RGBColor& color) {
    char x[8];
    sprintf(x, "%02x%02x%02x", color.r, color.g, color.b);
    return x;
  }

  static string rgbaColorToHex(const ml::RGBColor& color) {
    char x[10];
    sprintf(x, "%02x%02x%02x%02x", color.r, color.g, color.b, color.a);
    return x;
  }

  static bool hexToRGBColor(const string& hex, ml::RGBColor* pColor);

  ColorIndex() { }
  explicit ColorIndex(const util::Index<string>& labels) : m_labels(labels) {
    m_colors.resize(labels.size());
    for (int i = 0; i < labels.size(); ++i) {
      m_colors[i] = ml::RGBColor(ml::ColorUtils::colorById<ml::vec4f>(i));
    }
  }

  size_t size() const {
    return m_colors.size();
  }

  const vec<ml::RGBColor>& colors() const {
    return m_colors;
  }

  const ml::RGBColor& color(int index) const {
    return m_colors[index];
  }

  const ml::RGBColor& color(const string& label) const {
    int index = m_labels.indexOf(label);
    return m_colors[index];
  }

  const ml::RGBColor& color(const string& label, bool add = false) {
    int index = m_labels.indexOf(label);
    if (index < 0) {
      index = addLabel(label);
    } 
    return m_colors[index];
  }

  const util::Index<string>& labels() const {
    return m_labels;
  }

  // Set index to color
  void setColor(int i, const ml::RGBColor& color) {
    m_colors[i] = color;
  }

  //! Add label with next color
  int addLabel(const string& label) {
    int i = m_labels.add(label);
    size_t ncolors = m_colors.size();
    if (i >= ncolors) {
      m_colors.resize(i+1);
      m_colors[i] = ml::RGBColor(ml::ColorUtils::colorById<ml::vec4f>(i));
    }
    return i;
  }

  //! Add label with specified color
  int addLabel(const string& label, const ml::RGBColor& color) {
    int i = m_labels.add(label);
    size_t ncolors = m_colors.size();
    if (i >= ncolors) {
      m_colors.resize(i + 1);
    }
    m_colors[i] = color;
    return i;
  }

  //! Clear
  void clear() {
    m_colors.clear();
    m_labels.clear();
  }

  //! Save color legend to HTML
  void saveColorLegend(const string& filename) const;

  void saveColorLegend(ostream& os) const;

  //! Save color legend to CSV
  void saveCSV(const string& filename) const;

  //! Load color legend from CSV
  void loadCSV(const string& filename);

 private:
  util::Index<string> m_labels;
  vec<ml::RGBColor> m_colors;
};

}  // namespace vis
}  // namespace sg


