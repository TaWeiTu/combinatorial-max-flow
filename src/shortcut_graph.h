#pragma once

#include <tuple>
#include <vector>

#include "graph.h"

struct ShortcutGraph {
  Graph without_shortcut, shortcut;
  std::vector<int> levels;
  WeightT L;
  CapacityT scale;
  std::vector<WeightT> weights;
  std::vector<int> tau;  // topological order
  // map each edge in the shortcut graph to the original graph, or -1 if it is a
  // star edge.
  std::vector<Edge> edge_map;

  ShortcutGraph Reverse() const {
    auto res = *this;
    res.without_shortcut = res.without_shortcut.Reverse();
    res.shortcut = res.shortcut.Reverse();
    for (auto &a : res.tau) a = without_shortcut.n - a - 1;
    for (auto &[a, b, c] : res.star_edge_map) c = StarEdgeDirection(c ^ 1);
    for (auto &l : res.inv_star_edge_map)
      for (auto &a : l) std::swap(a[0], a[1]);
    return res;
  }

  enum StarEdgeDirection : uint8_t { kToStar = 0, kFromStar = 1 };

  // map each star edge to its level and its vertex; only entries corresponding
  // to star edges are meaningful.
  std::vector<std::tuple<int, Vertex, StarEdgeDirection>> star_edge_map;
  // map each (l, v) pair to its star edge.
  std::vector<std::vector<std::array<Edge, 2>>> inv_star_edge_map;

  ShortcutGraph() = default;
  ShortcutGraph(const Graph &g, std::vector<int> levels, CapacityT scale,
                bool skip_top_level);

  Edge StarEdge(int l, Vertex v, StarEdgeDirection dir) const {
    return inv_star_edge_map[l][v][dir];
  }
  bool IsStarVertex(Vertex v) const { return v >= without_shortcut.n; }
  bool IsStarEdge(Edge e) const { return e >= without_shortcut.m; }
  bool IsTopLevelStarEdge(Edge e) const {
    return IsStarEdge(e) && std::get<0>(star_edge_map[e]) == L;
  }
  bool IsTopLevelEdge(Edge e) const { return !IsStarEdge(e) && levels[e] == L; }
};
