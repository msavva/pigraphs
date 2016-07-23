#include "common.h"  // NOLINT

#include "stats/distributions.h"

#include <boost/math/distributions/beta.hpp>

#include "math/math.h"

namespace sg {
namespace stats {

std::random_device rd;
std::default_random_engine rg(rd());

double GaussianDistribution::sample() const {
  std::normal_distribution<> g(m_mu, m_s);  // TODO(MS): Cache
  return g(rg);
}

double GaussianDistribution::prob(const double& x) const {
  double k = 1 / (m_s * sqrt(math::constants::PI2));  // TODO(MS): Cache
  double v = - (x - m_mu) * (x - m_mu) / (2 * m_s * m_s);
  return k * exp(v);
}

double UniformDistribution::sample() const {
  return math::DEF_RAND.uniform_float(static_cast<float>(m_a), static_cast<float>(m_b));
}

double UniformDistribution::prob(const double& x) const {
  if (x >= m_a && x <= m_b) {
    return 1 / (m_b - m_a);
  } else {
    return 0.0;
  }
}

double BetaDistribution::sample() const {
  boost::math::beta_distribution<> bd(m_a, m_b);  // TODO(MS): Cache
  return quantile(bd, math::DEF_RAND.uniform_float_01());
}

double BetaDistribution::prob(const double& x) const {
  boost::math::beta_distribution<> bd(m_a, m_b);  // TODO(MS): Cache
  return pdf(bd, x);
}

VonMisesDistribution::VonMisesDistribution(double mu, double kappa): m_mu(mu), m_k(kappa) {
  m_logOffset = 0.5 * log(m_k) - m_k -0.5 * math::constants::LOGPI2;
}

//! rejection sampling scheme adapted from
//! http://stats.stackexchange.com/questions/156729/sampling-from-von-mises-fisher-distribution-in-python
double vm_rW(const double k, const double m) {
  const double dim = m - 1;
  BetaDistribution beta(dim / 2, dim /2);
  const double b = dim / (sqrt(4 * k * k + dim*dim) + 2 * k);
  const double x = (1 - b) / (1 + b);
  const double c = k * x + dim*log(1 - x * x);
  while (true) {
    const double z = beta.sample();
    const double w = (1 - (1+b)*z) / (1 - (1-b)*z);
    const double u = math::DEF_RAND.uniform_float_01();
    if (k*w + dim*log(1-x*w) - c >= log(u)) {
      return w;
    }
  }
}

double VonMisesDistribution::sample() const {
  if (m_k > 100) { return m_mu; }  // don't bother sampling if extremely concentrated
  const double dim = 1;
  const double w = vm_rW(m_k, dim + 1);
  const double sign = (math::DEF_RAND.uniform_float_01() < 0.5) ? 1 : -1;
  const double v = m_mu + (sign * math::constants::PI / 2);  // choose one of two perp points
  const double cosv = cos(v), sinv = sin(v), cosm = cos(m_mu), sinm = sin(m_mu);
  const double a = sqrt(1 - w*w), b = w;
  const double cosr = a*cosv + b*cosm, sinr = a*sinv + b*sinm;
  const double theta = atan2(sinr, cosr);
  return theta;
}

double VonMisesDistribution::logprob(const double& x) const {
  if (m_k < 0.1) {  // approximate low concentration as uniform
    return -math::constants::LOGPI2;
  } else if (m_k > 100) {  // approximate high concentration as delta function
    return (x == m_mu) ? std::numeric_limits<double>::infinity() : 0;
  }

  // use exponential approximation to I_0(m_k)
  return m_k * cos(x - m_mu) + m_logOffset;
}

double VonMisesDistribution::logprob(const double& x, double w) const {
  if (m_k < 0.1) {  // approximate low concentration as uniform
    return -math::constants::LOGPI2 + log(w);
  } else if (m_k > 100) {  // approximate high concentration as delta function
    return (x == m_mu) ? 1 : 0;
  }
  // use exponential approximation to I_0(m_k)
  return m_k * cos(x - m_mu) + m_logOffset + log(w);
}

}  // namespace stats
}  // namespace sg
