#include "common.h"  // NOLINT

#include "eval/metrics.h"

namespace sg {
namespace eval {

void SummedMetrics::operator+=(const SummedMetrics& o) {
  const auto avg = MetricsSet(o);
  Psum = Psum + avg.P;
  Rsum = Rsum + avg.R;
  F1sum = F1sum + avg.F1;
  apSum += avg.ap;
  ndcgSum += avg.ndcg;
  numSum++;  // add one for macro averaging since o is already summed
}

void SummedMetrics::operator+=(const MetricsSet& o) {
  Psum = Psum + o.P;
  Rsum = Rsum + o.R;
  F1sum = F1sum + o.F1;
  apSum += o.ap;
  ndcgSum += o.ndcg;
  numSum += o.num;
}

AverageMetrics SummedMetrics::getAverages() const {
  MetricsSet avg(*this);
  AverageMetrics o;
  const size_t N = avg.P.size() - 1;
  o.P = avg.P;
  o.R = avg.R;
  o.PatN = avg.P[N];
  o.RatN = avg.R[N];
  o.F1atN = avg.F1[N];
  o.mAP = avg.ap;
  o.ndcg = avg.ndcg;
  o.num = numSum;
  return o;
}

}  // namespace eval
}  // namespace sg
