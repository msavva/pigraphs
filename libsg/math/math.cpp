#include "common.h"  // NOLINT

#include "math/math.h"

#include <boost/math/special_functions/bessel.hpp>
#include <boost/math/special_functions/beta.hpp>

namespace sg {
namespace math {

double besselI(int v, double x) {
  if (x > 30) {  // hackishly avoid overflow exception by using approximation for large x
    return exp(x) / sqrt(constants::PI2 * x);
  } else {
    return boost::math::cyl_bessel_i(v, x);
  }
}

double beta(double a, double b) {
  return boost::math::beta(a, b);
}

}  // namespace math
}  // namespace sg
