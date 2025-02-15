#include "shortcut_graph.h"

#include <algorithm>

ShortcutGraph::ShortcutGraph(Graph g, std::vector<int> levels, CapacityT scale)
    : g(g),
      levels(levels),
      scale(scale),
      weights(g.m),
      edge_map(g.m),
      star_edge_map(g.m, std::make_tuple(-1, -1, kToStar)) {
  shortcut = g * scale;
  auto order = RespectingOrder(g, levels);
  for (Edge e : g.Edges()) {
    weights[e] = std::abs(order[g.tail[e]] - order[g.head[e]]);
    edge_map[e] = e;
  }

  const int L = *std::max_element(levels.begin(), levels.end());
  inv_star_edge_map.assign(L + 1, std::vector<std::array<Edge, 2>>(g.n));
  for (int l = 0; l <= L; ++l) {
    auto scc = g.EdgeSubgraph([&](Edge e) { return levels[e] <= l; }).SCC();
    std::vector<CapacityT> tail_capacity(g.n);
    for (Edge e : g.Edges()) {
      if (levels[e] == l && scc[g.tail[e]] == scc[g.head[e]]) {
        tail_capacity[g.tail[e]] += g.capacity[e];
      }
    }
    int k = *std::max_element(scc.begin(), scc.end());
    std::vector<Vertex> stars(k + 1);
    std::vector<int> size(k + 1);
    for (int j = 0; j <= k; ++j) stars[j] = g.AddVertex();
    for (Vertex v : g.Vertices()) size[scc[v]]++;
    for (Vertex v : g.Vertices()) {
      inv_star_edge_map[l][v][kToStar] =
          g.AddEdge(v, stars[scc[v]], tail_capacity[v]);
      edge_map.push_back(-1);
      star_edge_map.emplace_back(l, v, kToStar);
      weights.push_back(size[scc[v]]);
      inv_star_edge_map[l][v][kFromStar] =
          g.AddEdge(stars[scc[v]], v, tail_capacity[v]);
      edge_map.push_back(-1);
      weights.push_back(size[scc[v]]);
      star_edge_map.emplace_back(l, v, kFromStar);
    }
  }
}
