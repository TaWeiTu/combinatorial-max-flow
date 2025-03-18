#include "maximum_flow.h"

#include <algorithm>
#include <cassert>
#include <numeric>
#include <ranges>
#include <vector>

#include "expander_hierarchy.h"
#include "flow_util.h"
#include "graph.h"
#include "weighted_push_relabel.h"

namespace {

enum Direction { kForward, kBackward };

std::pair<Graph, std::vector<std::pair<Edge, Direction>>> ConstructResidual(
    const Graph& g, const std::vector<CapacityT>& flow) {
  Graph residual(g.n);
  std::vector<std::pair<Edge, Direction>> edge_map;
  for (auto e : g.Edges()) {
    auto [x, y, c] = g.EdgeInfo(e);
    if (flow[e] < c) {
      residual.AddEdge(x, y, c - flow[e]);
      edge_map.emplace_back(e, kForward);
    }
    if (flow[e] > 0) {
      residual.AddEdge(y, x, flow[e]);
      edge_map.emplace_back(e, kBackward);
    }
  }
  return std::make_pair(residual, edge_map);
}

void UpdateFlowFromResidual(
    std::vector<CapacityT>& flow, const Graph& residual,
    const std::vector<std::pair<Edge, Direction>>& edge_map,
    const std::vector<CapacityT>& flow_on_residual) {
  for (auto e : residual.Edges()) {
    auto [ee, dir] = edge_map[e];
    if (dir == kForward) flow[ee] += flow_on_residual[e];
    if (dir == kBackward) flow[ee] -= flow_on_residual[e];
  }
}

}  // namespace

// Implements [Algorithm 3, TODO: new paper] until the maximum flow is found
std::pair<CapacityT, std::vector<CapacityT>> MaximumFlow(const Graph& g,
                                                         Vertex s, Vertex t) {
  // TODO: should we also implement capacity scaling?

  std::vector<CapacityT> flow(g.m, 0);
  std::vector<CapacityT> st_demand(g.n, 0);
  // Divide infinity by 1'000'000'000 since we will scale it up by a lot.
  const CapacityT inf =
      std::numeric_limits<CapacityT>::max() / 1'000'000'000'000'000;
  st_demand[s] = inf;
  st_demand[t] = -inf;

  while (true) {
    // We repeatedly find a polylog approximation to the max flow in the
    // residual graph. This while loop thus runs O(polylog n) times at most

    // Step 1: calculate the residual graph from the current flow
    auto [residual, edge_map] = ConstructResidual(g, flow);

    // Step 2: build the expander hierarchy
    ExpanderHierarchy eh(residual);

    // Step 3: run weighted push relabel on shortcut graph
    auto [flow_value, flow_on_sg, _cut] =
        WeightedPushRelabelOnShortcut(eh.GetShortcutGraph(), st_demand, 1);

    if (flow_value == 0) break;  // no flow in the residual graph, we are done

    // Step 4: unwrap the flow
    auto flow_on_residual = eh.Unfold(flow_on_sg);
    assert(std::ssize(flow_on_residual) == residual.m);

    // calculate (an upperbound) on the congestion of the flow
    CapacityT congest = 1;
    for (auto e : residual.Edges()) {
      auto c = (flow_on_residual[e] + residual.capacity[e] - 1) /
               residual.capacity[e];
      congest = std::max(congest, c);
    }

    // This flow has congestion 3 in the scaled up graph, so it has congestion
    // 3/psi overall in the original graph.
    // TODO: In our current flow-unfolding, we lose a factor more than 3, so
    // this is not quite true, if I understand correctly.
    // assert(std::ranges::all_of(residual.Edges(), [&](Edge e) {
    //  return 0 <= flow[e] && flow[e] <= 3 * eh.Scale() * residual.capacity[e];
    //}));

    // Now we round it back to be integral and feasible.
    flow_on_residual =
        FlowRoundingRoundedDown(residual, flow_on_residual, congest);
    assert(FlowToDemand(residual, flow_on_residual)[s] != 0);

    // Step 5: update the flow
    UpdateFlowFromResidual(flow, residual, edge_map, flow_on_residual);

    assert(std::ranges::all_of(g.Edges(),
                               [&](Edge e) {
                                 return 0 <= flow[e] &&
                                        flow[e] <= g.capacity[e];
                               }) &&
           "intermediate flow must be valid");
  }

  return {FlowToDemand(g, flow)[s], flow};
}
