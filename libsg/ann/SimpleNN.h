#pragma once

#include "ann/distance.h"
#include "libsg.h"  // NOLINT

namespace sg {
namespace ann {

template<class T1, class T2, class Pred = std::less<T2>>
struct sort_pair_second {
  bool operator()(const pair<T1,T2>&left, const pair<T1,T2>&right) {
    Pred p;
    return p(left.second, right.second);
  }
};

template <typename T, typename Tptr = T*, typename num_t = double>
class BruteForceNearestNeighbors
{
public:
  BruteForceNearestNeighbors(const Distance<T,num_t>& distance): m_distance(distance) {};
 
  //! Query for k closest points to given query.  
  //! Return indices and distances to points in out_pairs.
  //! @param[in] elements - Elements to look through 
  //! @param[in] query - Query point
  //! @param[out] out_pairs - Closest neighbors (sorted)
  //! @param[in] k - Number of neighbors to return
  //! @param[in] radius - Upper limit on the distance
  inline void queryKNN(const vec<Tptr>& elements, const T& query,
                       vec<pair<size_t, num_t>>* out_pairs, size_t k, size_t radius = 0) const {
      auto distanceFunc = [&](const T& t) { return m_distance.distance(query,t); };
      queryKNN(elements, distanceFunc, out_pairs, k, radius);
  }

  inline void queryKNNAve(const vec<Tptr>& elements, const vec<Tptr>& queries,
                          vec<pair<size_t, num_t>>* out_pairs, size_t k, size_t radius = 0) const {
      auto distanceFunc = [&](const T& t) { 
        num_t distSum = 0.0;
        for (size_t i = 0; i < queries.size(); i++) {
          distSum += m_distance.distance(*queries[i],t);
        }
        return distSum/queries.size();
      };
    queryKNN(elements, distanceFunc, out_pairs, k, radius);
  }

  inline void queryKNNMin(const vec<Tptr>& elements, const vec<Tptr>& queries,
                          vec<pair<size_t, num_t>>* out_pairs, size_t k, size_t radius = 0) const {
      auto distanceFunc = [&](const T& t) { 
        num_t minDist = std::numeric_limits<num_t>::max();
        for (size_t i = 0; i < queries.size(); i++) {
          num_t dist = m_distance.distance(*queries[i],t);
          if (dist < minDist) {
            minDist = dist;
          }
        }
        return minDist;
      };
      queryKNN(elements, distanceFunc, out_pairs, k, radius);
  }

  //! Query for k closest points to given query.  
  //! Return indices and distances to points in out_pairs.
  //! @param[in] elements - Elements to look through 
  //! @param[in] distanceFunc - Distance function for each element
  //! @param[out] out_pairs - Closest neighbors (sorted)
  //! @param[in] k - Number of neighbors to return
  //! @param[in] radius - Upper limit on the distance
  inline void queryKNN(const vec<Tptr>& elements, const std::function<num_t(const T&)>& distanceFunc,
                       vec<pair<size_t, num_t>>* out_pairs, size_t k, size_t radius = 0) const {
      out_pairs->clear();
      if (k > 0) {
        out_pairs->reserve(k);
      }
      for (size_t i = 0; i < elements.size(); i++) {
        num_t dist = distanceFunc(*elements[i]);
        // Skip if dist is larger than the radius...
        if (radius > 0 && dist > radius) continue;  
        if (k > 0 && out_pairs->size() > k) {
          num_t max_dist = out_pairs->front().second;
          if (dist < max_dist) {
            // Remove largest element
            std::pop_heap(out_pairs->begin(), out_pairs->end(), cmp);
            out_pairs->pop_back();
            // Add new element
            out_pairs->push_back(std::make_pair(i, dist));
            std::push_heap(out_pairs->begin(), out_pairs->end(), cmp);
          }
        } else {
          // Add new element
          out_pairs->push_back(std::make_pair(i, dist));
          std::push_heap(out_pairs->begin(), out_pairs->end(), cmp);
        }
      }
      std::sort_heap(out_pairs->begin(), out_pairs->end(), cmp);
  }

private:
  const Distance<T,num_t>& m_distance;
  sort_pair_second<size_t,num_t> cmp;
};

}  // namespace ann
}  // namespace sg


