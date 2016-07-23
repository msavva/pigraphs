#include "common.h"  // NOLINT

#include "graph/alignment.h"
#include "util/util.h"

#include <boost/heap/binomial_heap.hpp>

using namespace boost::heap;

namespace sg {
namespace graph {
namespace alignment {

//! Represents a partial alignment from a set of objects (S1) to another set of objects (S2)
template <typename T1, typename T2, typename num_t = double>
class PartialAlignment : public Alignment<T1,T2,num_t> {
public:
  // Remaining options to consider
  typedef std::map<T1, std::vector<T2>> RemainingT;
  RemainingT remaining;

  void clear() {
    aligned.clear();
    score = 0;
    remaining.clear();
  }

  void append(T1 t1, T2 t2, num_t s) {
    //aligned.emplace_back({t1,t2,s});
    //std::sort(aligned.begin(), aligned.end());
    //score += s;
    //remaining[t1].erase(std::find(remaining[t1].begin(), remaining.end(), t2));
  }

};

template <typename T1, typename T2, typename num_t = double>
struct PartialAlignmentScoreComparator {
  typedef PartialAlignment<T1, T2, num_t> PartialT;
  bool operator()(const PartialT &p1, const PartialT &p2) {
    return p1.score > p2.score;
  }
};

template <typename T1, typename T2, typename num_t = double>
class PartialAlignmentsRange {
public: 
  typedef PartialAlignment<T1, T2, num_t> PartialT;
  //typedef PartialAlignmentComparator<T1, T2, num_t> PartialComparatorT;

  double scoreMin;
  double scoreMax;
  std::vector<PartialT> partials;

  PartialAlignmentsRange(const PartialT& p) {
    scoreMin = p.score;
    scoreMax = p.score;
    partials.push_back(p);
  }
  void add(const PartialAlignment<T1, T2, num_t>& p) { 
    scoreMin = math.min(scoreMin, p.score);
    scoreMax = math.max(scoreMax, p.score);

    // Check if not same then push back
    partials.push_back(p); 
  }
  
};

template <typename T1, typename T2, typename num_t = double>
struct PartialAlignmentsRangeComparator {
  typedef PartialAlignmentsRange<T1, T2, num_t> PartialRangeT;
  bool operator()(const PartialRangeT &p1, const PartialRangeT &p2) {
    return p1.scoreMax > p2.scoreMax;
  }
};

template <typename T1, typename T2, typename num_t = double>
class PartialAlignments {
public:
  // Queue of alignments we are to consider
  typedef PartialAlignment<T1,T2,num_t> PartialT;
  typedef PartialAlignmentScoreComparator<T1, T2, num_t> PartialScoreComparatorT;
  //typedef binomial_heap<PartialT, boost::heap::compare<PartialScoreComparatorT>> QueueT;
  typedef std::vector<PartialT> QueueT;

  QueueT alignments;

  size_t size() { return alignments.size(); }
  void clear() { alignments.clear(); }
  void add(const PartialAlignment<T1,T2,num_t>& p) { 
//    alignments.push(p); 
    alignments.push_back(p);
  }
  void add(const PartialAlignment<T1,T2,num_t>& p, T1 i1, T2 i2, num_t s) {
    PartialAlignment<T1,T2,num_t> p2 = p;
    p2.append(i1,i2,s);
//    alignments.push(p2); 
    alignments.push_back(p2); 
  }

  void getTopSorted(int limit, vec<PartialAlignment<T1,T2,num_t>>* pOut) {
    int sz = (int) size();
    int n = (limit > 0)? std::min(limit, sz) : sz;
    pOut->clear();
    pOut->reserve(n);

    int i = 0;
//    for (auto it = alignments.ordered_begin(); it != alignments.ordered_end(); ++it) {
    for (auto it = alignments.begin(); it != alignments.end(); ++it) {
//      pOut->emplace_back(it);
      i++;
      if (i >= n) break;
    }    
  }

  void getTopAlignments(int limit, vec<Alignment<T1,T2,num_t>>* pOut) {
    int sz = (int)size();
    int n = (limit > 0) ? std::min(limit, sz) : sz;
    pOut->clear();
    pOut->reserve(n);

    int i = 0;
//    for (auto it = alignments.ordered_begin(); it != alignments.ordered_end(); ++it) {
    for (auto it = alignments.begin(); it != alignments.end(); ++it) {
//      pOut->emplace_back(it);
      i++;
      if (i >= n) break;
    }
  }
};

typedef PartialAlignment<int, int, double> PartialAlignmentInt;
typedef PartialAlignments<int, int, double> PartialAlignmentsInt;
typedef vec<PartialAlignmentInt> VecPartialAlignmentInt;

void generateTopAlignments(const BeamAlignerParams& params,
                           int s1, int s2, int limit, VecAlignmentInt* pAlignments);

void getEmptyPartialAlignment(const BeamAlignerParams& params, int s1, int s2, PartialAlignmentInt* partial) {
  partial->clear();
  for (int i = 0; i < s1; i++) {
    partial->remaining[i].reserve(s2);
    for (int j = 0; j < s2; j++) {
      partial->remaining[i][j] = j;
    }
  }
}

void generateTopAlignments(const BeamAlignerParams& params, 
                           PartialAlignmentInt* initial, int limit,
                           VecAlignmentInt* pAlignments) {
  // Generate top n alignments using a beam.... this will not necessarily be best
  VecPartialAlignmentInt* pSorted = new VecPartialAlignmentInt();
  PartialAlignmentsInt* pCandidates = new PartialAlignmentsInt();
  pCandidates->add(*initial);
  int beamSize = (int) (limit * params.beamFactor);
  bool done = false;
  while (!done) {
    pCandidates->clear();
    done = true;
    // Trim candidates
    pCandidates->getTopSorted(beamSize, pSorted);
    for (const auto& candidate : *pSorted) {
      const auto& i1Options = sg::util::make_iterator_pair(candidate.remaining.begin(), candidate.remaining.end());
      for (const auto& i1Option : i1Options) {
        const auto& i1 = i1Option.first;
        const auto& i2Options = i1Option.second;
        bool added = false;
        double i1UnalignedScore = std::numeric_limits<double>::quiet_NaN(); 
        for (const auto& i2 : i2Options) {
          const auto& s = params.scorer(i1,i2);
          if (i2 == -1) {
            i1UnalignedScore = s;
          } else if (s >= params.minSingleAlignThreshold) {
            pCandidates->add(candidate, i1, i2, s);
            // Indicate that this option was considered regardless of whether it was actually added or not
            added = true;            
          }
        }
        if (!std::isnan(i1UnalignedScore) && params.allowEmpty && !(params.avoidEmptyIfPossible && added)) {
          pCandidates->add(candidate, i1, -1, i1UnalignedScore);
        }
      }
      done = false;
    }
  }
  pCandidates->getTopAlignments(limit, pAlignments);
  delete pSorted;
  delete pCandidates;
}

void generateTopAlignments(const BeamAlignerParams& params, int s1, int s2, int limit,
                           VecAlignmentInt* pAlignments) {
  PartialAlignmentInt initial;
  getEmptyPartialAlignment(params, s1, s2, &initial);
  generateTopAlignments(params, &initial, limit, pAlignments);
}


}  // namespace alignment
}  // namespace graph
}  // namespace sg
