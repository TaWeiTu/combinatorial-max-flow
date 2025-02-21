#pragma once

#include "graph.h"

// Calculate the maximum s-t flow in O(n^2 polylog(n)) time.
// returns the value of the flow and a flow vector on the edges.
std::pair<CapacityT, std::vector<CapacityT>> MaximumFlow(const Graph& g,
                                                         Vertex s, Vertex t);
