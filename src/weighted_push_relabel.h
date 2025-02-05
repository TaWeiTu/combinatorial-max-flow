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
// returns flow vector and a cut
// positive demand is source, negative demand is sink
std::tuple<CapacityT, std::vector<CapacityT>, std::vector<bool>>
WeightedPushRelabelOnShortcut(ShortcutGraph g, std::vector<CapacityT> demand,
                              CapacityT kappa, CapacityT scale);
