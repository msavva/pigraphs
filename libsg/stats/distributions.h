#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace stats {

//! Generic probability distribution over type T
template <typename T>
class Distribution {
 public:
  virtual ~Distribution() { }
  //! Takes n samples from this Distribution and returns the sample with the highest logprob
  virtual pair<T,double> sampleMax(int n) const {
    pair<T,double> m;
    m.second = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
      T s = sample();
      double lp = logprob(s);
      if (lp > m.second) {
        m.first = s;
        m.second = lp;
      }
    }
    return m;
  };
  //! Type/name of distribution
  virtual string type() const = 0;
  //! Returns a sample from this Distribution
  virtual T sample() const = 0;
  //! Returns mean value
  virtual T mean() const = 0;
  //! Returns probability density at x under this Distribution
  virtual double prob(const T& x) const { return exp(logprob(x)); }
  //! Returns log(probability density) at x under this Distribution
  virtual double logprob(const T& x) const { return log(prob(x)); }
  //! Returns probability at x estimated by width*pdf(x)
  virtual double prob(const T& x, double width) const { return prob(x) * width; }
  //! Returns log(probability) at x estimated by width*pdf(x)
  virtual double logprob(const T& x, double width) const { return logprob(x) + log(width); }
};

class GaussianDistribution : public Distribution<double> {
 public:
  explicit GaussianDistribution(double mu = 0, double sigma = 1) : m_mu(mu), m_s(sigma) { }
  string type() const override { return "Gaussian"; }
  double sample() const override;
  double mean() const override { return m_mu; }
  double sigma() const { return m_s; }
  double variance() const { return m_s * m_s; }
  double prob(const double& x) const override;

 private:
  double m_mu, m_s;
};
inline ostream& operator<<(ostream& os, const GaussianDistribution& d) {
  os << "{ \"type\":\"" << d.type() << "\", \"mu\":" << d.mean() << ", \"sigma\":" << d.sigma() << " }";
  return os;
}

class UniformDistribution : public Distribution<double> {
 public:
  explicit UniformDistribution(double a = 0, double b = 1) : m_a(a), m_b(b) { }
  string type() const override { return "Uniform"; }
  double sample() const override;
  double mean() const override { return (m_a + m_b) / 2; }
  double min() const { return m_a; }
  double max() const { return m_b; }
  double prob(const double& x) const override;

 private:
  double m_a, m_b;
};
inline ostream& operator<<(ostream& os, const UniformDistribution& d) {
  os << "{ \"type\":\"" << d.type() << "\", \"min\":" << d.min() << ", \"max\":" << d.max() << " }";
  return os;
}

class BetaDistribution : public Distribution<double> {
 public:
  explicit BetaDistribution(double alpha = 1, double beta = 1) : m_a(alpha), m_b(beta) { }
  string type() const override { return "Beta"; }
  double sample() const override;
  double mean() const override { return m_a / (m_a + m_b); }
  double alpha() const { return m_a; }
  double beta() const { return m_b; }
  double prob(const double& x) const override;

 private:
  double m_a, m_b;
};
inline ostream& operator<<(ostream& os, const BetaDistribution& d) {
  os << "{ \"type\":\"" << d.type() << "\", \"a\":" << d.alpha() << ", \"b\":" << d.beta() << " }";
  return os;
}

//! von Mises distribution over unit circle (p=2)
class VonMisesDistribution : public Distribution<double> {
 public:
  explicit VonMisesDistribution(double mu = 0, double kappa = 1);
  string type() const override { return "VMD"; }
  double sample() const override;
  double mean() const override { return m_mu; }
  double concentration() const { return m_k; }

  double prob(const double& x, double w) const override { return exp(logprob(x, w)); }
  double logprob(const double& x) const override;
  double logprob(const double& x, double w) const override;

 private:
  double m_mu, m_k, m_logOffset;
};
inline ostream& operator<<(ostream& os, const VonMisesDistribution& d) {
  os << "{ \"type\":\"" << d.type() << "\", \"mu\":" << d.mean() << ", \"kappa\":" << d.concentration() << " }";
  return os;
}

}  // namespace stats
}  // namespace sg
