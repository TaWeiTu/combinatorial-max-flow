#pragma once

#include <utility>

#include "graph.h"
#include "shortcut_graph.h"

// weighted push-relabel
// implements [Algorithm 1, https://arxiv.org/pdf/2406.03648]
// positive demand is source, negative demand is sink
//
// returns:
//  1. value of flow
//
//  2. a flow vector of length g.m, assigning flow[e] flow to edge e
//
//  3. the residual demand vector, of length g.n.  residual_demand[u] =
//    demand[u] - sum_{(u,v) in E} flow[u,v] + sum_{(v,u) in E} flow[v,u]
//
// Running time is O(m + h*n + (sum_e h/w[e])*log n)
std::tuple<CapacityT, std::vector<CapacityT>, std::vector<CapacityT>>
WeightedPushRelabel(Graph g, std::vector<CapacityT> demand,
                    const std::vector<WeightT> w, WeightT h);

// weighted push-relabel on shortcut graph
// implements [Lemma 4.1, TODO: cite new paper]
//
// returns:
//   1. amount of flow routed (scaled by both sg.scale and kappa).
//
//   2. the flow vector: on a kappa-scaled version of the shortcut graph (which
//   is already scaled up by sg.scale).
//
//   3. a cut: vector of length sg.without_shortcut.n, where 1 denotes it is on
//   the source-side of the cut, and 0 on the sink-side. Note that the cut
//   vector does not include star-vertices.
//
std::tuple<CapacityT, std::vector<CapacityT>, std::vector<bool>>
WeightedPushRelabelOnShortcut(ShortcutGraph sg, std::vector<CapacityT> demand,
                              CapacityT kappa);

// Computes and returns a flow exactly routing the demand on a phi-expander. If
// the demand k-respects the degree of the expander, then the flow has value
// f(e) <= k * O(log n / phi) * c(e).
// The running time is O(n log^2 n / phi), given that the graph is actually a
// phi-expander.
std::vector<CapacityT> PushRelabelOnExpander(Graph expander,
                                             std::vector<CapacityT> demand);
