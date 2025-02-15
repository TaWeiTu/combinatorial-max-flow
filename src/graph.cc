#include "graph.h"

#include <ranges>

std::ostream& operator<<(std::ostream& os, const Graph& g) {
  os << g.n << " " << g.m << "\n";
  for (Edge e : g.Edges()) {
    os << g.tail[e] << " -> " << g.head[e] << "\n";
  }
  return os;
}

Graph Graph::operator*(CapacityT scale) const {
  Graph g = *this;
  for (auto& c : g.capacity) c = c * scale;
  return g;
}

Subgraph Graph::VertexSubgraph(const std::vector<Vertex>& s) const {
  Subgraph sg;
  for (Vertex v : s) {
    sg.inv_vertex_map[v] = sg.g.AddVertex();
    sg.vertex_map.push_back(v);
  }
  std::vector<Edge> edge_list;
  for (Vertex v : s) {
    for (Edge e : out_edges[v]) {
      if (sg.inv_vertex_map.find(head[e]) != sg.inv_vertex_map.end()) {
        edge_list.push_back(e);
        sg.inv_edge_map[e] =
            sg.g.AddEdge(sg.inv_vertex_map[tail[e]], sg.inv_vertex_map[head[e]],
                         capacity[e]);
        sg.edge_map.push_back(e);
      }
    }
  }
  return sg;
}

std::vector<int> Graph::SCC() const {
  std::vector<bool> vis(n);
  std::vector<Vertex> order;
  std::vector<int> scc_id(n, -1);

  auto Dfs = [&](auto self, Vertex v) -> void {
    vis[v] = true;
    for (Edge e : out_edges[v]) {
      Vertex u = head[e];
      if (!vis[u]) self(self, u);
    }
    order.push_back(v);
  };

  for (Vertex v : Vertices()) {
    if (!vis[v]) Dfs(Dfs, v);
  }

  auto RevDfs = [&](auto self, Vertex v, int id) -> void {
    scc_id[v] = id;
    for (Edge e : in_edges[v]) {
      Vertex u = tail[e];
      if (scc_id[u] == -1) self(self, u, id);
    }
  };

  int num_scc = 0;
  for (Vertex v : std::ranges::reverse_view(order)) {
    if (scc_id[v] == -1) RevDfs(RevDfs, v, num_scc++);
  }
  return scc_id;
}

namespace {

std::vector<int> RespectingOrderInternal(const Graph& g,
                                         const std::vector<int>& levels,
                                         int max_l) {
  if (max_l == 0) {
    return std::ranges::to<std::vector<int>>(std::ranges::views::iota(0, g.n));
  }
  Graph lower_level_subgraph =
      g.EdgeSubgraph([&levels, max_l](Edge e) { return levels[e] < max_l; });
  auto sublevels = std::ranges::to<std::vector<int>>(
      levels | std::views::filter([max_l](int l) { return l < max_l; }));
  auto scc = lower_level_subgraph.SCC();
  const int num_scc = *std::max_element(scc.begin(), scc.end()) + 1;
  std::vector<std::vector<int>> comps(num_scc);
  for (Vertex v : lower_level_subgraph.Vertices()) {
    comps[scc[v]].push_back(v);
  }

  auto SubList = [&](const auto& lst, const auto& idx) {
    std::decay_t<decltype(lst)> sublst;
    for (int i : idx) sublst.push_back(lst[i]);
    return sublst;
  };

  std::vector<int> order(g.n);
  int offset = 0;
  for (int i = 0; i < num_scc; ++i) {
    auto component = lower_level_subgraph.VertexSubgraph(comps[i]);
    auto component_levels = SubList(sublevels, component.edge_map);
    auto subgraph_order =
        RespectingOrderInternal(component.g, component_levels, max_l - 1);
    for (int j = 0; j < component.g.n; ++j) {
      order[comps[i][j]] = subgraph_order[j] + offset;
    }
    offset += component.g.n;
  }
  return order;
}

}  // namespace

std::vector<int> RespectingOrder(const Graph& g,
                                 const std::vector<int>& levels) {
  return RespectingOrderInternal(
      g, levels, *std::max_element(levels.begin(), levels.end()) + 1);
}
