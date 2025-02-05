#pragma once

#include <iostream>
#include <map>
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
  std::pair<Graph, std::vector<Edge>> VertexSubgraph(
      const std::vector<Vertex>& s) const {
    Graph subgraph;
    std::map<Vertex, Vertex> order;
    for (Vertex v : s) {
      order[v] = subgraph.AddVertex();
    }
    std::vector<Edge> edge_list;
    for (Vertex v : s) {
      for (Edge e : out_edges[v]) {
        if (order.find(head[e]) != order.end()) {
          edge_list.push_back(e);
          subgraph.AddEdge(order[tail[e]], order[head[e]], capacity[e]);
        }
      }
    }
    return std::make_pair(subgraph, edge_list);
  }

  // Return a vector of length g.n which indicates the SCC each vertex belongs
  // to. The SCCs are numbered according to some valid topological order.
  std::vector<int> SCC() const;
};

std::ostream& operator<<(std::ostream& os, const Graph& g);

// Compute a vertex ordering of `g` that respects the hierarchy induced by
// `levels`.
//
// TODO: Put this function here for now. Can be moved to an appropriate file in
// the future.
std::vector<int> RespectingOrder(const Graph& g,
                                 const std::vector<int>& levels);
