#pragma once

#ifndef Q_MOC_RUN  // This is necessary to prevent Qt MoC from barfing on Boost
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#endif

#include "libsg.h"  // NOLINT
#include "util/util.h"
#include "geo/geo.h"

namespace sg {
namespace mesh {
namespace ContactDeterminator {

// Typedefs of PartHandle types
// TODO(ms): This really should not to be redefined here
typedef size_t PartHandle;
typedef vec<PartHandle> VecPartHandle;
typedef vec<VecPartHandle> VecVecPartHandle;

// Represents the set of all samples each belonging to a corresponding part.
// Used for KD-tree based contact determination
struct SampleSet { geo::VecVec3f points; vec<PartHandle> partIndices; };

// Represents a set of "contact points" between two parts
struct ContactSet { set<size_t> pointIndices; };

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
        boost::no_property, ContactSet, SampleSet> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

// Given Parts in V return a Graph edges between parts that have sample points
// of another part within threshold * OBB diagonal distance from one of their
// sample points
template <typename Part>
Graph contactGraph(const vec<Part*>& V, float threshold) {
  // Initialization
  util::println("Constructing adjacency graph");
  const size_t nV = V.size();
  Graph G;
  vec<Graph::vertex_descriptor> V2(nV);
  for (size_t i = 0; i < nV; i++) { V2[i] = boost::add_vertex(G); }
  geo::KNNWorkspace ws;

  // Aggregate all part samples and create a K-D tree
  geo::VecVec3f& samples = G[boost::graph_bundle].points;
  vec<size_t>& samplePartIndices = G[boost::graph_bundle].partIndices;
  for (size_t i = 0; i < nV; i++) {
    samples.insert(end(samples), std::begin(V[i]->samples),
                   std::end(V[i]->samples));
    for (size_t j = 0; j < V[i]->samples.size(); j++) {
      samplePartIndices.push_back(i);
    }
  }
  ann::ANN<geo::VecVec3f, float>* kdTree = new ann::ANN<geo::VecVec3f, float>(samples, 10);

  // Iterate over samples and find pairs from different parts within threshold
  for (size_t i = 0; i < samples.size(); i++) {
    const geo::Vec3f& sample = samples[i];
    const size_t partIdx = samplePartIndices[i];
    const Part* part = V[partIdx];
    const float maxDist = part->obb.diagonalLength() * threshold;
    kdTree->queryRadius(sample, maxDist, &ws.out_pairs);

    for (auto pair : ws.out_pairs) {
      const size_t otherPartIdx = samplePartIndices[pair.first];
      if (otherPartIdx != partIdx) {  // Ignore points from same part
        // && !part->obb.contains(samples[pair.first])  // Ignore contained
        auto eExists = edge(V2[partIdx], V2[otherPartIdx], G);
        Edge e;
        if (eExists.second) {
          e = eExists.first;
        } else {
          auto eAdded = boost::add_edge(V2[partIdx], V2[otherPartIdx], G);
          assert(eAdded.second);
          e = eAdded.first;
          cout << "Contact " << partIdx << "-" << otherPartIdx << endl;
        }
        G[e].pointIndices.insert(pair.first);
      }
    }
  }

  delete kdTree;

  return G;
};

// Find connected components in contact Graph G and return as vector of
// component indices. Value of vector at i-th index is assigned cluster index
inline vec<int> connectedComponents(const Graph& G) {
  vec<int> comp(num_vertices(G));
  int numComponents = connected_components(G, &comp[0]);
  cout << "Number of components: " << numComponents << endl;
  for (size_t i = 0; i < comp.size(); ++i) {
    cout << "Node " << i << " in component " << comp[i] << endl;
  }
  return comp;
}

// Return whether any of the parts in a are in same adjacency connected comp.
// as any of the parts in b
inline bool contactPairExists(const VecPartHandle& a, const VecPartHandle& b,
                              const Graph& G) {
  bool adj = false;
  for (const PartHandle i : a) {
    for (const PartHandle j : b) {
      adj |= edge(i, j, G).second;
      //cout << i << "," << j << (adj ? "Adj" : "NonAdj") << endl;
    }
  }
  return adj;
}

// Group parts by connected component in a filtered G where only vertices
// corresponding to the parts are kept. Return map from component index to
// vector containing parts of each connected component
inline map<int, VecPartHandle>
groupByConnectedComponent(const VecPartHandle& parts, const Graph& G) {
  // Create filtered graph
  Graph G2;
  const size_t nParts = parts.size();
  vec<Graph::vertex_descriptor> V(nParts);
  for (size_t i = 0; i < nParts; i++) { V[i] = boost::add_vertex(G2); }
  for (size_t i = 0; i < nParts; i++) {
    for (size_t j = 0; j < nParts; j++) {
      if (edge(parts[i], parts[j], G).second) {
        boost::add_edge(V[i], V[j], G2);
      }
    }
  }

  // Get component indices
  vec<int> compIdx(nParts);
  connected_components(G2, &compIdx[0]);

  // Group by component
  map<int, VecPartHandle> byComponent;
  for (size_t i = 0; i < nParts; i++) {
    byComponent[compIdx[i]].push_back(parts[i]);
  }

  return byComponent;
}

// Return all contact points between two parts
inline geo::VecVec3f getContactPoints(const PartHandle i, const PartHandle j,
                                      const Graph& G) {
  geo::VecVec3f contacts;
  const geo::VecVec3f& points = G[boost::graph_bundle].points;
  const auto eExists = edge(i, j, G);
  if (eExists.second) {
    const set<size_t>& pointIndices = G[eExists.first].pointIndices;
    for (const size_t i : pointIndices) { contacts.push_back(points[i]); }
  }
  return contacts;
}

}  // namespace ContactDeterminator
}  // namespace mesh
}  // namespace sg


