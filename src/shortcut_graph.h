#pragma once

#include <tuple>
#include <vector>

#include "graph.h"

struct ShortcutGraph {
  Graph g, shortcut;
  CapacityT scale;
  std::vector<int> levels;
  std::vector<WeightT> weights;
  // map each edge in the shortcut graph to the original graph, or -1 if it is a
  // star edge.
  std::vector<Edge> edge_map;

  enum StarEdgeDirection { kToStar, kFromStar };

  // map each star edge to its level and its vertex; only entries corresponding
  // to star edges are meaningful.
  std::vector<std::tuple<int, Vertex, StarEdgeDirection>> star_edge_map;
  // map each (l, v) pair to its star edge.
  std::vector<std::vector<std::array<Edge, 2>>> inv_star_edge_map;

  ShortcutGraph() = default;
  ShortcutGraph(Graph g, std::vector<int> levels, CapacityT scale);

  Edge StarEdge(int l, Vertex v, StarEdgeDirection dir) const {
    return inv_star_edge_map[l][v][dir];
  }
};
