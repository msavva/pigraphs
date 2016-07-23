#pragma once

#include "libsg.h"  // NOLINT
#include "graph/Graph.h"
#include "math/math.h"
#include "util/Index.h"

namespace sg {
namespace graph {

//! Namespace containing algorithms for computing alignment between graphs
namespace alignment {

// Represents an alignment between two elements and their score
template <typename T1, typename T2, typename num_t = double>
struct ScoredPair {
  T1 t1;
  T2 t2;
  num_t score;

  friend int icmpPairLtScoreGt(const ScoredPair<T1, T2, num_t>& a, const ScoredPair<T1, T2, num_t>& b) {
    if (a.t1 < b.t1) return 1;
    if (a.t1 > b.t1) return -1;
    if (a.t2 < b.t2) return 1;
    if (a.t2 > b.t2) return -1;
    if (a.score > b.score) return 1;
    if (a.score < b.score) return -1;
    return 0;
  }

  friend bool cmpPairLtScoreGt(const ScoredPair<T1, T2, num_t>& a, const ScoredPair<T1, T2, num_t>& b) {
    int res = icmpPairLtScoreGt(a,b);
    return (res <= 0);
  }

  friend int icmpScoreGtPairLt(const ScoredPair<T1, T2, num_t>& a, const ScoredPair<T1, T2, num_t>& b) {
    if (a.score < b.score) return -1;
    if (a.score > b.score) return 1;
    if (a.t1 < b.t1) return -1;
    if (a.t1 > b.t1) return 1;
    if (a.t2 < b.t2) return -1;
    if (a.t2 > b.t2) return 1;
    return 0;
  }

  friend bool cmpScoreGtPairLt(const ScoredPair<T1, T2, num_t>& a, const ScoredPair<T1, T2, num_t>& b) {
    int res = icmpScoreGtPairLt(a,b);
    return (res <= 0);
  }

};




// Represents an alignment between two sets
template <typename T1, typename T2, typename num_t = double>
struct Alignment {
  vec<ScoredPair<T1,T2,num_t>> aligned;
  num_t score;

  friend int cmpScoreGt(const Alignment<T1, T2, num_t>& a, const Alignment<T1, T2, num_t>& b) {
    if (a.score < b.score) return false;
    if (a.score > b.score) return true;
    return false;
  }

  friend int cmpScoreGtAlignedGt(const Alignment<T1, T2, num_t>& a, const Alignment<T1, T2, num_t>& b) {
    if (a.score < b.score) return false;
    if (a.score > b.score) return true;
    return cmpAlignedGt(a.aligned, b.aligned);
  }

  friend int icmpAlignedGt(const vec<ScoredPair<T1,T2,num_t>>& a, const vec<ScoredPair<T1,T2,num_t>>& b) {
    if (a.size < b.size) return -1;
    if (a.size > b.size) return 1;
    for (int i = 0; i < a.size; i++) {
      int res = icmpPairLtScoreGt(a[i], b[i]);
      if (res != 0) return res;
    }
    return 0;
  }

  friend bool cmpAlignedGt(const vec<ScoredPair<T1, T2, num_t>>& a, const vec<ScoredPair<T1, T2, num_t>>& b) {
    int res = icmpAligned(a,b);
    return (res <= 0);
  }

};

template <typename Graph1T, typename Graph2T, typename num_t = double>
class GraphAlignment {
  typedef typename Graph1T::vertex  vertex1;
  typedef typename Graph1T::edge    edge1;
  typedef Graph1T                   graph1;

  typedef typename Graph2T::vertex  vertex2;
  typedef typename Graph2T::edge    edge2;
  typedef Graph2T                   graph2;

  Alignment<vertex1, vertex2> alignedVertices;
  Alignment<edge1, edge2> alignedEdges;
  num_t score;
};

typedef Alignment<int, int, double> AlignmentInt;
typedef vec<AlignmentInt> VecAlignmentInt;

struct BeamAlignerParams {
  typedef std::function<double(int, int)> ScoringFn;
  ScoringFn scorer;
  double minSingleAlignThreshold;
  bool   allowEmpty;
  bool   avoidEmptyIfPossible;
  //bool   enforceLimit;
  double beamFactor = 1.5;
};

template <typename T1, typename T2, typename num_t = double>
class BeamAligner {
public:
  BeamAlignerParams params;
  void generateTopAlignments(const sg::util::Index<T1> index1, const sg::util::Index<T2> index2,
                             int limit, vec<Alignment<T1, T2>>* pAlignments) {
    vec<Alignment<T1,T2>> alignmentsInt;
    int s1 = index1.size;
    int s2 = index2.size;
    generateTopAlignments(params, s1, s2, limit, &alignmentsInt);
    pAlignments.clear();
    pAlignments.resize(alignmentsInt.size);
    for (size_t i = 0; i < alignmentsInt.size; i++) {
      const AligmentInt& alignmentInt = alignmentsInt.at(i);
      Alignment<T1,T2>& outAlignment = pAlignments[i];
      // Transform from an alignment over ints into an alignment over T1 and T2
      outAlignment.resize(alignmentInt.aligned.size);
      for (size_t j = 0; j < alignmentInt.aligned.size; j++) {
        outAlignment.aligned[j].t1 = index1[alignmentInt.aligned[j].t1];
        outAlignment.aligned[j].t2 = index2[alignmentInt.aligned[j].t2];
        outAlignment.aligned[j].score = alignmentInt.aligned[j].score
      }  
      outAlignment.score = score;
    }
  }
};

void generateTopAlignments(const BeamAlignerParams& params,
                           int s1, int s2, int limit, VecAlignmentInt* pAlignments);





}  // namespace alignment
}  // namespace graph
}  // namespace sg


