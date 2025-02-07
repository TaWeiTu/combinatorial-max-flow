#include "shortcut_graph.h"

#include <algorithm>

ShortcutGraph::ShortcutGraph(const Graph &g, const std::vector<int> &levels)
    : g_(g), levels_(levels) {}

std::pair<Graph, std::vector<WeightT>> ShortcutGraph::ToGraphAndWeights(
    CapacityT scale) const {
  Graph g = g_ * scale;
  auto order = RespectingOrder(g, levels_);
  std::vector<WeightT> weights(g_.m);
  for (Edge e : g_.Edges()) {
    weights[e] = std::abs(order[g_.tail[e]] - order[g_.head[e]]);
  }

  const int L = *std::max_element(levels_.begin(), levels_.end());
  for (int l = 0; l <= L; ++l) {
    auto scc = g_.EdgeSubgraph([&](Edge e) { return levels_[e] <= l; }).SCC();
    std::vector<CapacityT> tail_capacity(g_.n);
    for (Edge e : g_.Edges()) {
      if (levels_[e] == l && scc[g_.tail[e]] == scc[g_.head[e]]) {
        tail_capacity[g_.tail[e]] += g_.capacity[e];
      }
    }
    int k = *std::max_element(scc.begin(), scc.end());
    std::vector<Vertex> stars(k + 1);
    std::vector<int> size(k + 1);
    for (int j = 0; j <= k; ++j) stars[j] = g.AddVertex();
    for (Vertex v : g_.Vertices()) size[scc[v]]++;
    for (Vertex v : g_.Vertices()) {
      g.AddEdge(v, stars[scc[v]], tail_capacity[v]);
      weights.push_back(size[scc[v]]);
      g.AddEdge(stars[scc[v]], v, tail_capacity[v]);
      weights.push_back(size[scc[v]]);
    }
  }
  return std::make_pair(g, weights);
}
