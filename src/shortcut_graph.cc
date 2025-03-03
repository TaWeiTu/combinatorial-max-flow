#include "shortcut_graph.h"

#include <algorithm>
#include <ranges>

ShortcutGraph::ShortcutGraph(const Graph& g, std::vector<int> levels,
                             CapacityT scale, bool skip_top_level)
    : without_shortcut(g),
      shortcut(g * scale),
      levels(levels),
      L(levels.empty() ? -1 : std::ranges::max(levels) - skip_top_level),
      scale(scale),
      weights(g.m),
      tau(RespectingOrder(g, levels)),
      edge_map(g.m),
      star_edge_map(g.m, std::make_tuple(-1, -1, kToStar)) {
  for (Edge e : g.Edges()) {
    weights[e] = std::abs(tau[g.tail[e]] - tau[g.head[e]]);
    edge_map[e] = e;
  }

  inv_star_edge_map.assign(L + 1, std::vector<std::array<Edge, 2>>(g.n));
  // the loop skips l = L, since these are the top-level edge sets
  for (int l = 0; l <= L; ++l) {
    auto scc = g.EdgeSubgraph([&](Edge e) { return levels[e] <= l; }).SCC();
    std::vector<CapacityT> tail_capacity(g.n);
    for (Edge e : g.Edges()) {
      if (levels[e] == l && scc[g.tail[e]] == scc[g.head[e]]) {
        tail_capacity[g.tail[e]] += g.capacity[e];
      }
    }
    int k = std::ranges::max(scc);
    std::vector<Vertex> stars(k + 1);
    std::vector<int> size(k + 1);
    for (int j = 0; j <= k; ++j) stars[j] = shortcut.AddVertex();
    for (Vertex v : g.Vertices()) size[scc[v]]++;
    for (Vertex v : g.Vertices()) {
      // TODO: possible optimization is to omit 0-weight edges
      inv_star_edge_map[l][v][kToStar] =
          shortcut.AddEdge(v, stars[scc[v]], tail_capacity[v]);
      edge_map.push_back(-1);
      star_edge_map.emplace_back(l, v, kToStar);
      weights.push_back(size[scc[v]]);
      inv_star_edge_map[l][v][kFromStar] =
          shortcut.AddEdge(stars[scc[v]], v, tail_capacity[v]);
      edge_map.push_back(-1);
      weights.push_back(size[scc[v]]);
      star_edge_map.emplace_back(l, v, kFromStar);
    }
  }
}
