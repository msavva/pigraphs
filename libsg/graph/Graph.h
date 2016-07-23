#pragma once

#include <boost/graph/adjacency_list.hpp>

namespace sg {
namespace graph {

// Templatized abstract graph class wrapping Boost Graph Library adjacency_list
// (see: http://www.boost.org/doc/libs/1_56_0/libs/graph/doc/adjacency_list.html)
// VertexBundleT stored at vertices, EdgeBundleT at edges, and GraphBundleT for the entire graph
template <typename VertexBundleT, typename EdgeBundleT, typename GraphBundleT>
class Graph {
 public:
  //! Boost graph type
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                         VertexBundleT, EdgeBundleT, GraphBundleT> boost_graph;
  //! Pointer to vertex
  typedef typename boost_graph::vertex_descriptor vertex;
  //! Pointer to edge
  typedef typename boost_graph::edge_descriptor edge;
  //! Pair of vertices (source,target)
  typedef pair<vertex, vertex> vertexpair;
  //! Type of data stored at vertices
  typedef VertexBundleT VertexBundleT;
  //! Type of data stored at edges
  typedef EdgeBundleT EdgeBundleT;
  //! Type of data stored at edges
  typedef GraphBundleT GraphBundleT;

  virtual ~Graph() { }

  //! Add a vertex to this Graph and return the descriptor pointing to it
  vertex addVertex() {
    return boost::add_vertex(m_G);
  }

  //! Add an Edge from a to b in this Graph and returns descriptor pointing to it.
  //! If the edge already exists, it is returned without creating a new one.
  edge addEdge(const vertex& a, const vertex& b) {
    return boost::add_edge(a, b, m_G).first;
  }

  //! Get vector of out edges connected to vertex v
  vec<edge> getOutEdges(const vertex& v) const {
    vec<edge> edges;
    const auto itPair = boost::out_edges(v, m_G);
    for (auto it = itPair.first; it != itPair.second; ++it) {
      edges.push_back(*it);
    }
    return edges;
  }

  //! Get vector of in edges connected to vertex v
  vec<edge> getInEdges(const vertex& v) const {
    vec<edge> edges;
    const auto itPair = boost::in_edges(v, m_G);
    for (auto it = itPair.first; it != itPair.second; ++it) {
      edges.push_back(*it);
    }
    return edges;
  }

  //! Return the VertexBundleT at v
  VertexBundleT& getVertexBundle(const vertex& v) {
    return m_G[v];
  }

  //! Return the VertexBundleT stored at v
  const VertexBundleT& getVertexBundle(const vertex& v) const {
    return m_G[v];
  }

  //! Return the EdgeBundleT stored at e
  EdgeBundleT& getEdgeBundle(const edge& e) {
    return m_G[e];
  }

  //! Return the EdgeBundleT stored at e
  const EdgeBundleT& getEdgeBundle(const edge& e) const {
    return m_G[e];
  }

  //! Return GraphBundleT stored for this Graph
  GraphBundleT& getGraphBundle() {
    return m_G[boost::graph_bundle];
  }

  //! Return GraphBundleT stored for this Graph
  const GraphBundleT& getGraphBundle() const {
    return m_G[boost::graph_bundle];
  }

  //! Returns edge corresponding to given vertex pair (source,target)
  edge vertPairToEdge(const vertexpair& vp) const {
    const auto edgeExistsPair = boost::edge(vp.first, vp.second, m_G);
    assert(edgeExistsPair.second);  // edge must already exist
    return edgeExistsPair.first;
  }

  //! Return vertexpair (source,target) for given edge e
  vertexpair edgeToVertPair(const edge& e) const {
    return std::make_pair(boost::source(e, m_G), boost::target(e, m_G));
  }

  //! Converts edges to corresponding vertex pairs
  vec<vertexpair> edgesToVertPairs(const vec<edge>& edges) const {
    const size_t numEdges = edges.size();
    vec<vertexpair> pairs(numEdges);
    for (size_t i = 0; i < numEdges; ++i) {
      pairs[i] = edgeToVertPair(edges[i]);
    }
    return pairs;
  }

  //! Converts vertex pairs to corresponding edges
  vec<edge> vertPairsToEdges(const vec<vertexpair>& pairs) const {
    const size_t numPairs = pairs.size();
    vec<edge> edges(numPairs);
    for (size_t i = 0; i < numPairs; ++i) {
      edges[i] = vertPairToEdge(pairs[i]);
    }
    return edges;
  }

  //! Write GraphML representation out into ostream
  virtual void writeGraphML(ostream& os) = 0;  // NOLINT

 protected:
  typedef Graph<VertexBundleT, EdgeBundleT, GraphBundleT> GraphT;
  boost_graph m_G;

 private:
  // boost serialization support
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    ar & m_G;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    m_G.clear();  // Why is this necessary?!!!
    ar & m_G;
  }
};

}  // namespace graph
}  // namespace sg


