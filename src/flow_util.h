#pragma once

#include <map>
#include <utility>
#include <vector>

#include "graph.h"
#include "link_cut_tree.h"
#include "types.h"

using MultiCommodityDemand = std::map<std::pair<Vertex, Vertex>, CapacityT>;

// Flow decomposition
//
// Given a flow f in G, decompose into a multi-commodity demand D supported on
// O(m) pairs that f routes.
// After that, given any subdemand D' of D, we can compute a flow f' routing D'
// such that f' is entry-wise upper-bounded by f.
class FlowDecomposition {
 public:
  FlowDecomposition(const Graph& g, const std::vector<CapacityT>& flow);
  std::vector<CapacityT> Route(const MultiCommodityDemand& subdemand);

  const MultiCommodityDemand& Demand() const { return demand_; }

 private:
  std::map<std::pair<Vertex, Vertex>, CapacityT> demand_;
  Graph g_;
  std::vector<CapacityT> flow_;
};

// Flow rounding
//
// Given a flow f such that the demand d it routes is a multiple of some
// integral scale s, return a flow f' such that f' <= ceil(f/s) that routes d/s.
std::vector<CapacityT> FlowRoundingExact(const Graph& g,
                                          const std::vector<CapacityT>& flow,
                                          CapacityT scale);
// When the demand is not a multiple of s, the flow f' <= ceil(f/s) only routes
// floor(d/s).
std::vector<CapacityT> FlowRoundingRoundedDown(
    const Graph& g, const std::vector<CapacityT>& flow, CapacityT scale);
