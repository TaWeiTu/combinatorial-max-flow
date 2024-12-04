#pragma once

#include <ranges>
#include <vector>

#include "types.h"

struct Graph {
  int n = 0, m = 0;

  std::vector<std::vector<Edge>> out_edges;
  std::vector<std::vector<Edge>> in_edges;
  std::vector<CapacityT> capacity;
  std::vector<Vertex> head, tail;

  Edge AddEdge(Vertex t, Vertex h, CapacityT c = 0) {
    out_edges[t].push_back(m);
    in_edges[h].push_back(m);
    tail.push_back(t);
    head.push_back(h);
    capacity.push_back(c);
    return m++;
  }
  Vertex AddVertex() {
    out_edges.emplace_back();
    in_edges.emplace_back();
    return n++;
  }
  static Graph FromEdgeList(
      int n, const std::vector<std::tuple<Vertex, Vertex, CapacityT>>& edges) {
    Graph g;
    while (g.n < n) g.AddVertex();
    for (auto [x, y, c] : edges) g.AddEdge(x, y, c);
    return g;
  }

  auto Vertices() const { return std::ranges::views::iota(0, n); }
  auto Edges() const { return std::ranges::views::iota(0, m); }
  std::tuple<Vertex, Vertex, CapacityT> EdgeInfo(Edge e) const {
    return {tail[e], head[e], capacity[e]};
  }

  Graph Residual(const std::vector<CapacityT>& flow) const;
};
