
#include "weighted_push_relabel.h"

#include <cassert>
#include <limits>
#include <queue>

#include "graph.h"
#include "naive_link_cut_tree.h"

using namespace std;

namespace {

enum Direction { kNone = 0, kForward, kBackward };
using DirectedEdge = pair<Edge, Direction>;
const DirectedEdge kMissing = {-1, kNone};
const CapacityT kFlowMissing = -9;

struct V {
  CapacityT min_cap = std::numeric_limits<CapacityT>::max();
  DirectedEdge bottleneck = kMissing;
  friend V operator+(const V &a, const V &b) {
    return a.min_cap <= b.min_cap ? a : b;
  }
};

struct U {
  CapacityT inc = 0;
  static U Compose(const U &f, const U &g) { return U{.inc = f.inc + g.inc}; };
  static V Apply(const U &u, const V &v) {
    return V{.min_cap = v.min_cap + u.inc, .bottleneck = v.bottleneck};
  };
};

}  // namespace

pair<CapacityT, vector<CapacityT>> WeightedPushRelabel(Graph g,
                                                       vector<CapacityT> demand,
                                                       const vector<WeightT> w,
                                                       WeightT h) {
  assert(g.m == ssize(w));
  assert(g.n == ssize(demand));

  vector<vector<vector<Edge>>> edges_at_height(g.n,
                                               vector<vector<Edge>>(9 * h + 1));
  for (Edge e : g.Edges()) {
    if (g.tail[e] == g.head[e] || g.capacity[e] == 0) continue;
    assert(w[e] > 0);
    for (WeightT x = 0; x <= 9 * h; x += w[e]) {
      edges_at_height[g.tail[e]][x].emplace_back(e);
      edges_at_height[g.head[e]][x].emplace_back(e);
    }
  }

  queue<Vertex> todo, sources;
  for (Vertex v : g.Vertices()) {
    if (demand[v] >= 0) todo.emplace(v);
    if (demand[v] > 0) sources.emplace(v);
  }

  vector<WeightT> label(g.n, 0);
  vector<CapacityT> flow(g.m, 0);
  vector<bool> dead(g.n, false);
  vector<Direction> is_admissible(g.m, kNone);
  vector<queue<DirectedEdge>> admissible_out(g.n);
  vector<DirectedEdge> parent(g.n, kMissing);

  // TODO use actual fast link cut tree implementation
  auto link_cut_tree = NaiveLinkCutTree<U, V>(g.n);

  auto Head = [&](DirectedEdge e) -> Vertex {
    return e.second == kForward ? g.head[e.first] : g.tail[e.first];
  };

  auto FlowOnEdge = [&](Edge e) -> CapacityT {
    auto [x, y, c] = g.EdgeInfo(e);
    if (flow[e] != kFlowMissing) return flow[e];
    if (parent[x].first == e)
      return c - link_cut_tree.QueryParentEdge(x).min_cap;
    if (parent[y].first == e) return link_cut_tree.QueryParentEdge(y).min_cap;
    assert(false && "flow value not in tree");
  };

  // Mark an edge e as admissible / inadmissible
  // dir = kNone for inadmissible
  // dir = kForward to mark forward(e) as admissible
  // dir = kBackward to mark backward(e) as admissible
  auto SetAdmissible = [&](Edge e, Direction dir) {
    if (is_admissible[e] == dir) return;
    is_admissible[e] = dir;
    if (dir == kForward) admissible_out[g.tail[e]].emplace(e, dir);
    if (dir == kBackward) admissible_out[g.head[e]].emplace(e, dir);

    for (auto v : {g.tail[e], g.head[e]}) {
      while (!empty(admissible_out[v])) {
        auto [f, d] = admissible_out[v].front();
        if (is_admissible[f] == d) break;
        admissible_out[v].pop();
      }
      pair<Edge, Direction> new_parent =
          empty(admissible_out[v]) ? kMissing : admissible_out[v].front();

      if (new_parent == parent[v]) continue;

      if (parent[v] != kMissing) {
        auto [e, dir] = parent[v];
        auto value = link_cut_tree.QueryParentEdge(v);
        assert(value.bottleneck == parent[v]);
        assert(flow[e] == kFlowMissing);
        flow[e] =
            dir == kForward ? g.capacity[e] - value.min_cap : value.min_cap;
        link_cut_tree.CutParent(v);
        todo.emplace(v);
      }

      parent[v] = new_parent;
      if (parent[v] != kMissing) {
        auto [e, dir] = parent[v];
        V value{dir == kForward ? g.capacity[e] - flow[e] : flow[e], {e, dir}};
        link_cut_tree.Link(v, Head(value.bottleneck));
        link_cut_tree.SetParentEdge(v, value);
        flow[e] = kFlowMissing;
      }
    }
  };

  auto Relabel = [&](Vertex v) {
    ++label[v];
    if (label[v] > 9 * h) {
      dead[v] = true;
      return;
    }
    for (Edge e : edges_at_height[v][label[v]]) {
      auto [x, y, c] = g.EdgeInfo(e);
      CapacityT f = FlowOnEdge(e);

      if (label[x] - label[y] >= 2 * w[e] && c - f > 0)
        SetAdmissible(e, kForward);
      else if (label[y] - label[x] >= 2 * w[e] && f > 0)
        SetAdmissible(e, kBackward);
      else
        SetAdmissible(e, kNone);
    }
  };

  auto DropWhile = [](auto &v, auto &&pred) -> std::optional<Vertex> {
    while (!empty(v) && pred(v.front())) v.pop();
    if (empty(v)) return {};
    return v.front();
  };
  auto NextNeedRelabel = [&] {
    return DropWhile(todo,
                     [&](auto v) { return dead[v] || parent[v] != kMissing; });
  };
  auto NextSource = [&] {
    return DropWhile(sources,
                     [&](auto v) { return dead[v] || demand[v] == 0; });
  };

  CapacityT flow_value = 0;

  while (true) {
    while (auto ov = NextNeedRelabel()) Relabel(*ov);

    if (auto os = NextSource()) {
      // Augment along s -> t path
      Vertex s = *os, t = link_cut_tree.GetRoot(s);
      auto c_augment = min(
          {demand[s], -demand[t], link_cut_tree.QueryPathToRoot(s).min_cap});

      assert(c_augment > 0);
      flow_value += c_augment;
      link_cut_tree.UpdatePathToRoot(s, U{.inc = -c_augment});
      demand[s] -= c_augment;
      demand[t] += c_augment;
      if (demand[t] == 0) todo.emplace(t);

      while (true) {  // mark all saturated edges as inadmissible
        auto v = link_cut_tree.QueryPathToRoot(s);
        if (v.min_cap > 0) break;
        assert(v.min_cap == 0 && v.bottleneck != kMissing);
        SetAdmissible(v.bottleneck.first, kNone);
        s = Head(v.bottleneck);
      }
    } else
      break;
  }

  // make sure to remove edges from lct to compute their flow values
  for (Edge e : g.Edges()) SetAdmissible(e, kNone);

  return {flow_value, flow};
}
