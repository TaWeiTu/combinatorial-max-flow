#include "graph.h"

// weighted push-relabel
// implements [Algorithm 1, https://arxiv.org/pdf/2406.03648]

// returns flow vector of length g.m
// positive demand is source, negative demand is sink
std::pair<CapacityT, std::vector<CapacityT>> WeightedPushRelabel(
    Graph g, std::vector<CapacityT> demand, const std::vector<WeightT> w,
    WeightT h);
