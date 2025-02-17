#pragma once

#include <iostream>
#include <map>
#include <ranges>
#include <vector>

#include "types.h"

struct Subgraph;

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

  Graph() = default;
  Graph(int n) : n(n) {}

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
  Graph operator*(CapacityT scale) const;

  template <typename F>
  Graph EdgeSubgraph(F&& pred) const {
    std::vector<std::tuple<Vertex, Vertex, CapacityT>> edges;
    for (Edge e : Edges()) {
      if (pred(e)) edges.emplace_back(tail[e], head[e], capacity[e]);
    }
    return Graph::FromEdgeList(n, edges);
  }

  /// Return the vertex subgraph induced by vertices in `s` in O(vol(s)\log n)
  /// time. Also return which edge in the original graph each edge in the
  /// subgraph corresopnds to.
  Subgraph VertexSubgraph(const std::vector<Vertex>& s) const;

  // Return a vector of length g.n which indicates the SCC each vertex belongs
  // to. The SCCs are numbered according to some valid topological order.
  std::vector<int> SCC() const;
};

struct Subgraph {
  Graph g;
  std::vector<Vertex> vertex_map;
  std::vector<Edge> edge_map;
  std::map<Vertex, Vertex> inv_vertex_map;
  std::map<Edge, Edge> inv_edge_map;
};

std::ostream& operator<<(std::ostream& os, const Graph& g);

// Compute a vertex ordering of `g` that respects the hierarchy induced by
// `levels`.
//
// Returns a vector tau of length g.n, such that:
//   * For every level i, and SCC in G \ E_{>i}, the image tau(C) is contiguous.
//   * If u are v are in different level-i SCC but u can reach v in G \ E_{>i},
//   then tau[u] < tau[v].
//
// TODO: Put this function here for now. Can be moved to an appropriate file in
// the future.
std::vector<int> RespectingOrder(const Graph& g,
                                 const std::vector<int>& levels);
