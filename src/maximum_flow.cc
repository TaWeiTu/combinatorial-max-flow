
#include "maximum_flow.h"

#include <algorithm>
#include <cassert>
#include <numeric>
#include <ranges>
#include <vector>

#include "cut_matching_game.h"
#include "expander_hierarchy.h"
#include "weighted_push_relabel.h"

// Implements [Algorithm 3, TODO: new paper] until the maximum flow is found
std::pair<CapacityT, std::vector<CapacityT>> MaximumFlow(const Graph& g,
                                                         Vertex s, Vertex t) {
  // TODO: should we also implement capacity scaling?

  std::vector<CapacityT> flow(g.m, 0);
  std::vector<CapacityT> st_demand(g.n, 0);
  const CapacityT inf = std::numeric_limits<CapacityT>::max() / 3;
  st_demand[s] = inf;
  st_demand[t] = -inf;

  while (true) {
    // We repeatedly find a polylog approximation to the max flow in the
    // residual graph. This while loop thus runs O(polylog n) times at most

    // Step 1: calculate the residual graph from the current flow
    Graph residual(g.n);
    enum Direction { kForward, kBackward };
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

    // Step 2: build the expander hierarchy
    ExpanderHierarchy H(residual);

    // Step 3: run weighted push relabel on shortcut graph
    auto [flow_value, flow_on_sg, _cut] =
        WeightedPushRelabelOnShortcut(H.shortcut_graph_, st_demand, 1);

    if (flow_value == 0) break;  // no flow in the residual graph, we are done

    // Step 4: unwrap the flow
    auto flow_on_residual = H.UnwrapFlow(flow_on_sg);
    // TODO: remember that flow_on_sg is scaled up. somewhere we need to scale
    // down. Does this happen in H.Unwrap or here? When scaling down, we have to
    // have a rounded flow.

    // Step 5: update the flow
    assert(std::ssize(flow_on_residual) == residual.m);
    for (auto e : residual.Edges()) {
      auto [ee, dir] = edge_map[e];
      if (dir == kForward) flow[ee] += flow_on_residual[e];
      if (dir == kBackward) flow[ee] -= flow_on_residual[e];
    }

    assert(std::ranges::all_of(g.Edges(),
                               [&](Edge e) {
                                 return 0 <= flow[e] &&
                                        flow[e] <= g.capacity[e];
                               }) &&
           "intermediate flow must be valid");
  }

  CapacityT flow_value = 0;
  for (auto e : g.Edges()) {
    if (g.tail[e] == s) flow_value += flow[e];
    if (g.head[e] == s) flow_value -= flow[e];
  }

  return {flow_value, flow};
}
