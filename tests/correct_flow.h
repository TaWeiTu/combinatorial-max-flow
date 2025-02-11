#include "../src/graph.h"

namespace CorrectFlowImplementation {
using namespace std;
/**
 * from https://github.com/kth-competitive-programming/kactl
 * licence CC0
 **/
struct Dinic {
  struct Edge {
    int to, rev;
    CapacityT c, oc;
    CapacityT flow() { return max<CapacityT>(oc - c, 0); }  // if you need flows
  };
  vector<int> lvl, ptr, q;
  vector<vector<Edge>> adj;
  Dinic(int n) : lvl(n), ptr(n), q(n), adj(n) {}
  void addEdge(int a, int b, CapacityT c, CapacityT rcap = 0) {
    adj[a].push_back({b, static_cast<int>(ssize(adj[b])), c, c});
    adj[b].push_back({a, static_cast<int>(ssize(adj[a]) - 1), rcap, rcap});
  }

  CapacityT dfs(int v, int t, CapacityT f) {
    if (v == t || !f) return f;
    for (int& i = ptr[v]; i < ssize(adj[v]); i++) {
      Edge& e = adj[v][i];
      if (lvl[e.to] == lvl[v] + 1)
        if (CapacityT p = dfs(e.to, t, min(f, e.c))) {
          e.c -= p, adj[e.to][e.rev].c += p;
          return p;
        }
    }
    return 0;
  }

  CapacityT calc(int s, int t) {
    CapacityT flow = 0;
    q[0] = s;
    int L = 30;
    do {
      lvl = ptr = vector<int>(ssize(q));
      int qi = 0, qe = lvl[s] = 1;
      while (qi < qe && !lvl[t]) {
        int v = q[qi++];
        for (Edge e : adj[v])
          if (!lvl[e.to] && e.c >> (30 - L))
            q[qe++] = e.to, lvl[e.to] = lvl[v] + 1;
      }
      while (CapacityT p = dfs(s, t, LLONG_MAX)) flow += p;
    } while (lvl[t]);
    return flow;
  }
  bool leftOfMinCut(int a) { return lvl[a] != 0; }
};
CapacityT MaximumFlow(Graph g, const auto& demand) {
  Dinic mf(g.n + 2);
  int sink = g.n, source = g.n + 1;
  for (auto e : g.Edges()) {
    auto [x, y, c] = g.EdgeInfo(e);
    mf.addEdge(x, y, c);
  }
  for (auto v : g.Vertices()) {
    if (demand[v] < 0) mf.addEdge(v, sink, -demand[v]);
    if (demand[v] > 0) mf.addEdge(source, v, demand[v]);
  }
  return mf.calc(source, sink);
}
};  // namespace CorrectFlowImplementation
