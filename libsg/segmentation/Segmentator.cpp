#include "common.h"  // NOLINT

#include "segmentation/Segmentator.h"

#include <unordered_map>

#include "segmentation/MeshSegment.h"
#include "segmentation/segment-graph.h"  // Felzenswalb segmenter
#include "util/Params.h"
#include "util/util.h"
#include "util/index.h"

namespace sg {
namespace segmentation {

SegmentationParams::SegmentationParams(const sg::util::Params& p) {
  p.get("Segmentation.kThresh", &kthresh);
  p.get("Segmentation.segMinVerts", &minSegVerts);
  p.get("Segmentation.segMinDiag", &minSegDiag);
  p.get("Segmentation.segAspectCutoff", &segAspectCutoff);
  p.get("Segmentation.colorWeight", &colorWeight);
  p.get("Segmentation.constrainZup", &constrainZup);
}

MeshEdges getEdges(const ml::TriMeshf& mesh) {
  MeshEdges edges;
  const vec<ml::vec3ui>& indices = mesh.getIndices();
  for (int i = 0; i < mesh.getIndices().size(); i++) {
    UINT v1 = indices[i].x;
    UINT v2 = indices[i].y;
    UINT v3 = indices[i].z;
    edges.push_back(std::make_pair(v1, v2));
    edges.push_back(std::make_pair(v1, v3));
    edges.push_back(std::make_pair(v3, v2));
  }
  return edges;
}

vec<int> weldVertices(const ml::TriMeshf& mesh) {
  sg::util::Index<ml::vec3f> index;
  const vec<ml::TriMeshf::Vertex> verts = mesh.getVertices();
  const size_t nVerts = verts.size();
  vec<int> remap(nVerts);
  for (size_t i = 0; i < nVerts; i++) {
    remap[i] = index.add(verts[i].position);
  }
  return remap;
}

void createSegments(const ml::TriMeshf& mesh, const SegIndices& ids, const SegmentFilter& filter, bool constrainZup,
                    VecSegPtr* acceptedSegs, VecSegPtr* rejectedSegs) {
  const size_t nVertices = ids.size();
  vec<vec<size_t>> groups;
  std::unordered_map<int, vec<size_t>> map;
  for (size_t i = 0; i < nVertices; i++) { map[ids[i]].push_back(i); }
  sg::util::mapToVec(map, groups);
    
  int iSegTotal = 0, iSegAccepted = 0, iSegRejected = 0;
  acceptedSegs->clear();  rejectedSegs->clear();
  for (const vec<size_t>& els : groups) {
    SegmentPtr pSeg(new MeshSegment(mesh, els, iSegTotal++, constrainZup));
    if (filter(*pSeg)) {
      // Set the segment id to be equal to accepted segments' index
      pSeg->id = iSegAccepted;
      iSegAccepted++;
      acceptedSegs->push_back(pSeg);
    } else {
      // Set the segment id to be equal to negative of the rejected segments' index
      // Offset by -1 to make sure that the rejected segments are negative
      pSeg->id = -iSegRejected-1; 
      iSegRejected++;
      // We don't actually care about the rejectedSegs
      if (rejectedSegs != nullptr) {
        rejectedSegs->push_back(pSeg);
      }
    }
  }
}

void createSegments(const ml::TriMeshf& mesh, const SegIndices& ids, bool constrainZup,
                    VecSegPtr* acceptedSegs, VecSegPtr* rejectedSegs) {
  const size_t nVertices = ids.size();
  vec<vec<size_t>> groups;
  std::unordered_map<int, vec<size_t>> map;
  int maxSegId = -1, minSegId = 0;
  for (size_t i = 0; i < nVertices; i++) { 
    map[ids[i]].push_back(i); 
    if (ids[i] > maxSegId) { maxSegId = ids[i]; }
    if (ids[i] < minSegId) { minSegId = ids[i]; }
  }
  assert(maxSegId >= -1);
  assert(minSegId <= 0);

  acceptedSegs->clear();  
  acceptedSegs->resize(maxSegId + 1);
  rejectedSegs->clear();
  rejectedSegs->resize(-minSegId);
  for (auto& it = map.begin(); it != map.end(); ++it) {
    int iSegId = it->first;
    const auto& els = it->second;
    SegmentPtr pSeg(new MeshSegment(mesh, els, iSegId, constrainZup));
    if (iSegId >= 0) {
      // Set the segment id to be equal to accepted segments' index
      (*acceptedSegs)[iSegId] = pSeg;
    } else {
      // Set the segment id to be equal to negative of the rejected segments' index
      // Offset by -1 to make sure that the rejected segments are negative
      (*rejectedSegs)[-iSegId - 1] = pSeg;
    }
  }
}

SegIndices SegmentatorFelzenswalb::segment(const ml::TriMeshf& mesh) {
  cout << "Segmenting mesh with " << mesh.getVertices().size() << " vertices...";
  size_t N = mesh.getVertices().size();
  const MeshEdges& es = getEdges(mesh);
  size_t Ne = es.size();
  edge* edges = new edge[Ne];
  const auto& verts = mesh.getVertices();
  for (size_t i = 0; i < Ne; i++) {
    int a = es[i].first;
    int b = es[i].second;

    edges[i].a = a;
    edges[i].b = b;

    const ml::vec3f& n1 = verts[a].normal;
    const ml::vec3f& n2 = verts[b].normal;
    const ml::vec3f& p1 = verts[a].position;
    const ml::vec3f& p2 = verts[b].position;

    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    float dd = sqrt(dx * dx + dy * dy + dz * dz); dx /= dd; dy /= dd; dz /= dd;
    float dot = ml::vec3f::dot(n1, n2);
    float dot2 = n2.x * dx + n2.y * dy + n2.z * dz;
    float ww = 1.0f - dot;
    if (dot2 > 0) { ww = ww * ww; } // make it much less of a problem if convex regions have normal difference
    if (m_colorWeight > 0.0f) {
      const ml::vec4f& c1 = verts[a].color;
      const ml::vec4f& c2 = verts[b].color;
      //const float d = ml::vec3f::dist(c1, c2);
      //ww = (1 - m_colorWeight) * ww +  m_colorWeight * d;
      const ml::vec4f& c1hsl = ml::ColorUtils::rgbToHsl<ml::vec4f>(c1);
      const ml::vec4f& c2hsl = ml::ColorUtils::rgbToHsl<ml::vec4f>(c2);
      //const ml::vec4f& c1rgb = ml::ColorUtils::hslToRgb<ml::vec4f>(c1hsl);
      //const ml::vec4f& c2rgb = ml::ColorUtils::hslToRgb<ml::vec4f>(c2hsl);
      // Find difference in H and L, handling wrapping and make range [0,1]
      const float hdiff = 2.0f * std::min(fabsf(c1hsl[0] - c2hsl[0]), fabsf(c2hsl[0] - c1hsl[0] + 1));
      const float ldiff = 2.0f * std::min(fabsf(c1hsl[2] - c2hsl[2]), fabsf(c2hsl[2] - c1hsl[2] + 1));
      const float hldiff = sqrtf(hdiff*hdiff + ldiff*ldiff);
      ww = (1 - m_colorWeight) * ww +  m_colorWeight * hldiff;
    }

    edges[i].w = ww;
  }
  cout << "graph done... ";

  // Segment!
  universe* u = segment_graph(static_cast<int>(N), static_cast<int>(Ne), edges, m_kthr);
  cout << u->num_sets() << " initial segs. ";

  // Join small segments
  for (int j = 0; j < Ne; j++) {
    int a = u->find(edges[j].a);
    int b = u->find(edges[j].b);
    if ((a != b) && ((u->size(a) < m_minSegVerts) || (u->size(b) < m_minSegVerts))) {
      u->join(a, b);
    }
  }

  cout << " Done: " << u->num_sets() << " final segs" << endl;
  // Return segment indices
  SegIndices out(N);
  for (int q = 0; q < N; q++) {
    out[q] = u->find(q);
  }

  if (u) { delete u; }
  return out;
}

vec<SegIndices> Segmentator::segment(const vec<const ml::TriMeshf>& meshes) {
  vec<SegIndices> out;
  for (const ml::TriMeshf& mesh : meshes) {
    const SegIndices& segs = segment(mesh);
    //TODO(MS): Make unique by incrementing by previous aggregate max id
    out.push_back(segs);
  }
  return out;
}

}  // namespace segmentation
}  // namespace sg
