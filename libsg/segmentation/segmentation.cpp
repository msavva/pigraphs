#include "common.h"  // NOLINT

#include "segmentation/segmentation.h"
#include "segmentation/MeshSegment.h"
#include "segmentation/Segmentator.h"

#include "geo/OBB.h"

namespace sg {
namespace segmentation {

void saveSegIndicesCsv(const string& file, const SegIndices& segIndices)
{
  vec<SegIndices> vSegIndices;
  vSegIndices.push_back(segIndices);
  saveSegIndicesCsv(file,vSegIndices);
}

void saveSegIndicesCsv(const string& file, const vec<SegIndices>& segIndices) {
  ofstream ofs(file, std::ios::out);
  ofs << "meshIndex" << "," << "triIndex" << "," << "segIndex" << "\n";
  for (size_t mi = 0; mi < segIndices.size(); mi++) {
    const SegIndices s = segIndices[mi];
    for (size_t ti = 0; ti < s.size(); ti++) {
      ofs << mi << "," << ti << "," << s[ti] << "\n";
    }
  }
  ofs.close();
}

bool loadSegIndicesCsv(const string& filename, vec<SegIndices>* segIndices) {
  if (!ml::util::fileExists(filename)) { return false; }

  ifstream ifs(filename);
  string headerline;
  string line;
  string delimiter = ",";
  int iMeshIndex = 0;
  int iTriIndex = 1;
  int iSegIndex = 2;
  int defaultSegIndex = std::numeric_limits<int>::lowest();
  // Expect the CSV to have header
  if (getline(ifs, headerline)) {
    // Okay - we got the header line
    vec<string> header = ml::util::split(headerline, delimiter);
    // Check our header
    assert(header.size() == 3);
    assert(header[iMeshIndex] == "meshIndex");
    assert(header[iTriIndex] == "triIndex");
    assert(header[iSegIndex] == "segIndex");
    // Read rest of the CSV
    while (getline(ifs, line)) {
      vec<string> fields = ml::util::split(line, delimiter);
      assert(fields.size() == header.size());
      int meshIndex = std::stoi(fields[iMeshIndex]);
      assert(meshIndex >= 0);
      int triIndex = std::stoi(fields[iTriIndex]);
      assert(triIndex >= 0);
      int segIndex = std::stoi(fields[iSegIndex]);
      // segIndex can be negative

      // Makes sure our vector is large enough
      if (meshIndex >= segIndices->size()) {
        segIndices->resize(meshIndex + 1);
      }
      if (triIndex >= (*segIndices)[meshIndex].size()) {
        (*segIndices)[meshIndex].resize(triIndex + 1, defaultSegIndex);
      }
      (*segIndices)[meshIndex][triIndex] = segIndex;
    }
  }
  ifs.close();
  return true;
}

void storeSegmentIndices(ml::TriMeshf& mesh, const VecSegPtr& segments, const VecSegPtr& rejectedSegments) {
  // TODO(MS): This currently stores segment ids in texCoord[0]. Clean this hack up!
  // Convenient since when raycasting, we can get the intersected segId directly
  // Ideally, this shouldn't be stored in the texCoord[0] but some metadata field offered by the mesh
  vec<ml::TriMeshf::Vertex>& V = mesh.getVertices();
  for (size_t segIdx = 0; segIdx < segments.size(); segIdx++) {
    const auto& seg = segments[segIdx];
    for (const size_t vertexIdx : seg->elements) {
      V[vertexIdx].texCoord[0] = static_cast<float>(seg->id);
    }
  }
  for (size_t segIdx = 0; segIdx < rejectedSegments.size(); segIdx++) {
    const auto& seg = rejectedSegments[segIdx];
    for (const size_t vertexIdx : seg->elements) {
      V[vertexIdx].texCoord[0] = static_cast<float>(seg->id);  // NOTE: Rejected are marked negative
    }
  }
}

int vIdxToSegIdx(const ml::TriMeshf& mesh, int vIdx) {
  const auto& V = mesh.getVertices();
  return static_cast<int>(V[vIdx].texCoord[0]);
}

void getSegmentationColors(const ml::TriMeshf& mesh, int selectedSegIdx, vec<ml::vec4f>* colors) {
  if (colors->size() != mesh.getVertices().size()) {
    colors->resize(mesh.getVertices().size());
  }
  ml::util::fill(*colors, [&] (UINT64 i) {
    const int iSeg = vIdxToSegIdx(mesh, static_cast<int>(i));
    ml::vec4f c = ml::ColorUtils::colorById<ml::vec4f>(iSeg);
    if (iSeg == selectedSegIdx) {
      return 3.f * c;
    } else if (iSeg < 0) {
      return 0.2f * c;
    } else {
      return c;
    }
  });
}

void getSegmentationColors(const ml::TriMeshf& mesh, vec<int> selectedSegIdx, vec<ml::vec4f>* colors) {
  if (colors->size() != mesh.getVertices().size()) {
    colors->resize(mesh.getVertices().size());
  }
  set<int> selected(selectedSegIdx.begin(), selectedSegIdx.end());
  ml::util::fill(*colors, [&] (UINT64 i) {
    const int iSeg = vIdxToSegIdx(mesh, static_cast<int>(i));
    ml::vec4f c = ml::ColorUtils::colorById<ml::vec4f>(iSeg);
    if (selected.count(iSeg)) {
      return 3.f * c;
    } else if (iSeg < 0) {
      return 0.2f * c;
    } else {
      return c;
    }
  });
}

void getSegmentationColors(const ml::TriMeshf& mesh, const vec<double>& scores,
                           const ml::ColorGradient& colorGradient, vec<ml::vec4f>* colors){
  if (colors->size() != mesh.getVertices().size()) {
    colors->resize(mesh.getVertices().size());
  }
  ml::util::fill(*colors, [&] (UINT64 i) {
    const int iSeg = vIdxToSegIdx(mesh, static_cast<int>(i));
    if (iSeg >= 0 && iSeg < scores.size()) {
      double score = scores.at(iSeg);
      const auto cv = colorGradient.value(score);
      const ml::vec4f c = ml::vec4f(cv.r / 255.f, cv.g / 255.f, cv.b / 255.f, 1.f);
      return c;
    } else {
      const auto cv = ml::RGBColor::Gray;
      const ml::vec4f c = ml::vec4f(cv.r / 255.f, cv.g / 255.f, cv.b / 255.f, 1.f);
      return c;
    }
  });
}

geo::OBB computeSegsOBB(const VecConstSegPtr& segs) {
  vec<ml::vec3f> points;
  size_t totalPoints = 0;
  for (const auto& s : segs) {
    totalPoints += s->elements.size();
  }
  assert(totalPoints > 3);

  points.resize(totalPoints);
  int i = 0;
  for (const auto& s : segs) {
    const auto& V = s->mesh->getVertices();
    for (size_t vi : s->elements) {
      points[i] = V[vi].position;
      i++;
    }
  }
  assert(i == totalPoints);

  const geo::OBB obb(points, geo::OBB::FitOpts::CONSTRAIN_Z);
  return obb;
}

geo::BBox computeSegsBBox(const VecConstSegPtr& segs) {
  geo::BBox bbox;
  for (const ConstSegPtr& pSeg : segs) {
    bbox.extend(pSeg->obb()->toAABB());
  }
  return bbox;
}

}  // namespace segmentation
}  // namespace sg
