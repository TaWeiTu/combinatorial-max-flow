#include "graph.h"

#include <algorithm>
#include <numeric>
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

std::vector<int> RespectingOrder(const Graph& g,
                                 const std::vector<int>& levels) {
  int max_l = std::ranges::max(levels) + 1;
  std::vector<Vertex> order(g.n);
  std::iota(order.begin(), order.end(), 0);
  for (int i = 0; i < max_l; ++i) {
    auto scc = g.EdgeSubgraph([&](Edge e) { return levels[e] <= i; }).SCC();
    std::vector<std::vector<Vertex>> comps(std::ranges::max(scc) + 1);
    for (Vertex v : order) comps[scc[v]].push_back(v);
    order.clear();
    for (Vertex v : comps | std::views::join) order.push_back(v);
  }
  std::vector<int> tau(g.n);
  for (int i : std::views::iota(0, g.n)) tau[order[i]] = i;
  return tau;
}
