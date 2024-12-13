#pragma once

#include <map>
#include <vector>

#include "graph.h"
#include "link_cut_tree.h"
#include "types.h"

// Flow decomposition
//
// Given a flow f in G, decompose into a multi-commodity demand D supported on
// O(m) pairs that f routes.
// After that, given any subdemand D' of D, we can compute a flow f' routing D'
// such that f' is entry-wise upper-bounded by f.
class FlowDecomposition {
 public:
  FlowDecomposition(const Graph& g, const std::vector<CapacityT>& flow);
  std::vector<CapacityT> Route(
      const std::map<std::pair<Vertex, Vertex>, CapacityT>& subdemand);

  const std::map<std::pair<Vertex, Vertex>, CapacityT>& Demand() const {
    return multi_commodity_demand_;
  }

 private:
  std::map<std::pair<Vertex, Vertex>, CapacityT> multi_commodity_demand_;
  Graph g_;
  std::vector<CapacityT> flow_;

  std::vector<CapacityT> RouteInternal(
      std::map<std::pair<Vertex, Vertex>, CapacityT> subdemand,
      bool init = false);
};
