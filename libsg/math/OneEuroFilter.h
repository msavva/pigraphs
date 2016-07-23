#pragma once

#include "libsg.h"  // NOLINT

#include "math/math.h"
#include "geo/geo.h"

namespace sg {
namespace math {

/*
 * One Euro Filter for vectors and quaternions. Based on https://github.com/vrpn/vrpn
 **/

template<typename VecT = ml::vec3f, int DIM = 3, typename ScalarT = float>
class LowPassFilter {
 public:
  typedef ScalarT scalar_type;
  typedef VecT value_type;

  LowPassFilter() : _firstTime(true) { }

  value_type filter(const value_type& x, scalar_type alpha) {
    if (_firstTime) {
      _firstTime = false;
      _hatxprev = x;
    }

    value_type hatx;
    for (int i = 0; i < DIM; ++i) {
      hatx[i] = alpha * x[i] + (1 - alpha) * _hatxprev[i];
    }

    _hatxprev = hatx;
    return _hatxprev;
  }

  value_type hatxprev() {
    return _hatxprev;
  }

 private:
  bool _firstTime;
  value_type _hatxprev;
};

typedef LowPassFilter<> LowPassFilterVec;

template<typename VecT = ml::vec3f, int DIM = 3, typename ScalarT = float>
class VectorFilterable {
 public:
  typedef ScalarT scalar_type;
  typedef VecT value_type;
  typedef LowPassFilter<VecT, DIM, ScalarT> filter_type;

  static void setDxIdentity(value_type& dx) {
    for (int i = 0; i < DIM; ++i) {
      dx[i] = 0;
    }
  }

  static void computeDerivative(value_type& dx, const value_type& prev,
                                const value_type& current, scalar_type dt) {
    for (int i = 0; i < DIM; ++i) {
      dx[i] = (current[i] - prev[i]) / dt;
    }
  }

  static scalar_type computeDerivativeMagnitude(const value_type& dx) {
    scalar_type sqnorm = 0;
    for (int i = 0; i < DIM; ++i) {
      sqnorm += dx[i] * dx[i];
    }
    return sqrt(static_cast<ScalarT>(sqnorm));
  }
};

template<typename Filterable = VectorFilterable<> >
class OneEuroFilter {
 public:
  typedef typename Filterable::scalar_type scalar_type;
  typedef typename Filterable::value_type value_type;
  typedef typename Filterable::filter_type filter_type;

  OneEuroFilter(scalar_type mincutoff, scalar_type beta, scalar_type dcutoff)
    : _firstTime(true)
    , _mincutoff(mincutoff)
    , _dcutoff(dcutoff)
    , _beta(beta) { }

  OneEuroFilter() : _firstTime(true), _mincutoff(1), _dcutoff(1), _beta(0.5) { }

  void setMinCutoff(scalar_type mincutoff) {
    _mincutoff = mincutoff;
  }
  scalar_type getMinCutoff() const {
    return _mincutoff;
  }
  void setBeta(scalar_type beta) {
    _beta = beta;
  }
  scalar_type getBeta() const {
    return _beta;
  }
  void setDerivativeCutoff(scalar_type dcutoff) {
    _dcutoff = dcutoff;
  }
  scalar_type getDerivativeCutoff() const {
    return _dcutoff;
  }
  void setParams(scalar_type mincutoff, scalar_type beta, scalar_type dcutoff) {
    _mincutoff = mincutoff;
    _beta = beta;
    _dcutoff = dcutoff;
  }
  value_type filter(const value_type& x, scalar_type dt) {
    value_type dx;

    if (_firstTime) {
      _firstTime = false;
      Filterable::setDxIdentity(dx);
    } else {
      Filterable::computeDerivative(dx, _xfilt.hatxprev(), x, dt);
    }

    scalar_type derivative_magnitude = Filterable::computeDerivativeMagnitude(_dxfilt.filter(dx, alpha(dt, _dcutoff)));
    scalar_type cutoff = _mincutoff + _beta * derivative_magnitude;

    return _xfilt.filter(x, alpha(dt, cutoff));
  }

 private:
  static scalar_type alpha(scalar_type dt, scalar_type cutoff) {
    scalar_type tau = scalar_type(1) / (scalar_type(2) * constants::PI * cutoff);
    return scalar_type(1) / (scalar_type(1) + tau / dt);
  }

  bool _firstTime;
  scalar_type _mincutoff, _dcutoff;
  scalar_type _beta;
  filter_type _xfilt;
  filter_type _dxfilt;
};

typedef OneEuroFilter<> OneEuroFilterVec;

class LowPassFilterQuat {
 public:
  LowPassFilterQuat() : _firstTime(true) { }

  geo::Quatf filter(const geo::Quatf& x, float alpha) {
    if (_firstTime) {
      _firstTime = false;
      _hatxprev = x;
    }
    const geo::Quatf hatx = _hatxprev.slerp(alpha, x);
    _hatxprev = hatx;
    return _hatxprev;
  }

  geo::Quatf hatxprev() {
    return _hatxprev;
  }

 private:
  bool _firstTime;
  geo::Quatf _hatxprev;
};

class QuatFilterable {
 public:
  typedef float scalar_type;
  typedef geo::Quatf value_type;
  typedef LowPassFilterQuat filter_type;

  static void setDxIdentity(value_type& dx) {
    dx.x() = dx.y() = dx.z() = 0;
    dx.w() = 1;
  }

  static void computeDerivative(value_type& dx, const value_type& prev,
                                const value_type& current, scalar_type dt) {
    scalar_type rate = 1.0 / dt;

    const geo::Quatf prevInv = prev.inverse();
    dx = current * prevInv;

    // nlerp instead of slerp
    dx.x() *= rate;
    dx.y() *= rate;
    dx.z() *= rate;
    dx.w() = dx.w() * rate + (1.0 - rate);
    dx.normalize();
  }

  // VIP: Assumes normalized quaternion for input dx (i.e., |w| <= 1)
  static scalar_type computeDerivativeMagnitude(const value_type& dx) {
    // Weirdly, acos below seems to error out on domain boundary cases
    // so we handle them manually instead
    float w = dx.w();
    if (w <= -1.0f) {
      return 2.0f * constants::PIf;
    } else if (w >= 1.0f) {
      return 0.f;
    } else {
      return 2.0f * std::acos(dx.w());
    }
  }
};

typedef OneEuroFilter<QuatFilterable> OneEuroFilterQuat;


}  // namespace math
}  // namespace sg


