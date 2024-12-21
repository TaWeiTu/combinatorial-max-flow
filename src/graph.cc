#include "graph.h"

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
    order.push_back(v); };

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
