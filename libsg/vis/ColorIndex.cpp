#include "common.h"  // NOLINT

#include "vis/ColorIndex.h"

#include "io/csv.h"
#include "io/io.h"
#include "util/util.h"

namespace sg {
namespace vis {

bool ColorIndex::hexToRGBColor(const string& hex, ml::RGBColor* pColor) {
  string v = util::trim(hex);
  if (v.length() == 6) {
    sscanf(v.c_str(), "%2hhx%2hhx%2hhx", 
           &pColor->r, &pColor->g, &pColor->b);
    pColor->a = 255;
    return true;
  } else if (v.length() == 8) {
    sscanf(v.c_str(), "%2hhx%2hhx%2hhx%2hhx", 
           &pColor->r, &pColor->g, &pColor->b, &pColor->a);
    return true;
  } else {
    SG_LOG_WARN << "Cannot parse hex as color " << hex;
    return false;
  }
}

void ColorIndex::saveColorLegend(const string& filename) const {
  // Create a color legend and saves it out to file
  sg::io::ensurePathToFileExists(filename);
  ofstream ofs(filename);
  ofs << "<!DOCTYPE html><html><body>" << endl;
  saveColorLegend(ofs);
  ofs << "</body></html>" << endl;
  ofs.close();
}

void ColorIndex::saveColorLegend(ostream& os) const {
  // Create a html table with colors
  os << "<table>" << endl;
  for (int i = 0; i < m_labels.size(); i++) {
    const ml::RGBColor& color = m_colors[i];
    os << "<tr>";
    os << "<td width=50 bgcolor=\"#" << rgbColorToHex(color) << "\"></td>";
    os << "<td>" << m_labels[i] << "</td>";
    os << "<tr>" << endl;
  }
  os << "</table>" << endl;
  os.flush();
}

void ColorIndex::saveCSV(const string& filename) const {
  sg::io::ensurePathToFileExists(filename);
  ofstream ofs(filename);
  // Saves the color index to a csv file
  // Assumes no quotes in label
  ofs << "index,label,color" << endl;
  for (int i = 0; i < m_labels.size(); i++) {
    ofs << i << "," << m_labels[i] << ","  << rgbaColorToHex(m_colors[i]) << endl;
  }
  ofs.close();  
}

void ColorIndex::loadCSV(const string& filename) {
  clear();
  using io::csv::CSVReader;
  CSVReader<3, io::csv::trim_chars<' '>, io::csv::double_quote_escape<',','\"'> > csv(filename);
  csv.read_header(io::csv::ignore_extra_column, "index", "label", "color");
  string index, label, color;
  ml::RGBColor rgbColor;
  while (csv.read_row(index, label, color)) {
    hexToRGBColor(color, &rgbColor);
    addLabel(label, rgbColor);
  }
}

}  // namespace vis
}  // namespace sg
