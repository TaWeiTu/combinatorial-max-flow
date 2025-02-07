#pragma once

#include <utility>

#include "graph.h"
#include "shortcut_graph.h"

// weighted push-relabel
// implements [Algorithm 1, https://arxiv.org/pdf/2406.03648]

// returns flow vector of length g.m
// positive demand is source, negative demand is sink
std::pair<CapacityT, std::vector<CapacityT>> WeightedPushRelabel(
    Graph g, std::vector<CapacityT> demand, const std::vector<WeightT> w,
    WeightT h);

// weighted push-relabel on shortcut graph
// implements [Lemma 4.1]
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

std::vector<CapacityT> PushRelabelOnExpander(Graph expander, int phi,
                                             std::vector<CapacityT> demand);
