#include "weighted_push_relabel.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <list>
#include <numeric>
#include <queue>

#include "graph.h"
#include "link_cut_tree.h"

namespace {

enum Direction { kNone = 0, kForward, kBackward };
using DirectedEdge = std::pair<Edge, Direction>;
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

auto DropWhile(auto &v, auto &&pred)
    -> std::optional<std::decay_t<decltype(v.front())>> {
  while (!empty(v) && pred(v.front())) v.pop();
  if (empty(v)) return {};
  return v.front();
};

}  // namespace

// implements [Algorithm 1, https://arxiv.org/pdf/2406.03648]
std::tuple<CapacityT, std::vector<CapacityT>, std::vector<CapacityT>>
WeightedPushRelabel(Graph g, std::vector<CapacityT> demand,
                    const std::vector<WeightT> w, WeightT h) {
  assert(g.m == ssize(w));
  assert(g.n == ssize(demand));

  std::vector edges_at_height(g.n, std::vector<std::vector<Edge>>(9 * h + 1));
  for (Edge e : g.Edges()) {
    if (g.tail[e] == g.head[e] || g.capacity[e] == 0) continue;
    assert(w[e] > 0);
    for (WeightT x = 0; x <= 9 * h; x += w[e]) {
      edges_at_height[g.tail[e]][x].emplace_back(e);
      edges_at_height[g.head[e]][x].emplace_back(e);
    }
  }

  std::queue<Vertex> todo, sources;
  for (Vertex v : g.Vertices()) {
    if (demand[v] >= 0) todo.emplace(v);
    if (demand[v] > 0) sources.emplace(v);
  }

  std::vector<WeightT> label(g.n, 0);
  std::vector<CapacityT> flow(g.m, 0);
  std::vector<bool> dead(g.n, false);
  std::vector<Direction> is_admissible(g.m, kNone);
  std::vector<std::queue<DirectedEdge>> admissible_out(g.n);
  std::vector<DirectedEdge> parent(g.n, kMissing);

  auto link_cut_tree = LinkCutTree<U, V>(g.n);

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

  auto NextNeedRelabel = [&] {
    return DropWhile(todo,
                     [&](auto v) { return dead[v] || parent[v] != kMissing; });
  };
  auto NextSource = [&] {
    return DropWhile(sources,
                     [&](auto v) { return dead[v] || demand[v] == 0; });
  };
  auto NextAdmissibleOut = [&](Vertex v) {
    return DropWhile(admissible_out[v], [&](auto e) {
      return is_admissible[e.first] != e.second;
    });
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
      DirectedEdge new_parent = NextAdmissibleOut(v).value_or(kMissing);
      if (new_parent == parent[v]) continue;

      if (parent[v] != kMissing) {  // remove old edge from tree
        auto [e, dir] = parent[v];
        assert(flow[e] == kFlowMissing);
        flow[e] = FlowOnEdge(e);
        link_cut_tree.CutParent(v);
        todo.emplace(v);
      }

      parent[v] = new_parent;
      if (parent[v] != kMissing) {  // add new edge to tree
        auto [e, dir] = parent[v];
        V data{dir == kForward ? g.capacity[e] - flow[e] : flow[e], {e, dir}};
        link_cut_tree.Link(v, Head(data.bottleneck), data);
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

  CapacityT flow_value = 0;

  while (true) {
    while (auto ov = NextNeedRelabel()) Relabel(*ov);

    if (auto os = NextSource()) {  // Augment along s -> t path
      Vertex s = *os, t = link_cut_tree.GetRoot(s);
      auto c_augment = std::min(
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
    } else {
      // make sure to remove edges from lct to compute their flow values
      for (Edge e : g.Edges()) SetAdmissible(e, kNone);
      return {flow_value, flow, demand};
    }
  }
}

std::vector<CapacityT> PushRelabelOnExpander(Graph expander, int inv_phi,
                                             std::vector<CapacityT> demand) {
  CapacityT total_demand = std::accumulate(
      demand.begin(), demand.end(), CapacityT(0),
      [](auto a, auto b) { return a + std::max<CapacityT>(b, 0); });
  std::vector<WeightT> weights(expander.m, 1);
  std::vector<CapacityT> flow(expander.m);
  // TODO: figure out the right h
  WeightT h = WeightT(10 * inv_phi * log2(expander.n));
  while (total_demand > 0) {
    auto [v, f, rd] = WeightedPushRelabel(expander, demand, weights, h);
    for (Edge e : expander.Edges()) flow[e] += f[e];
    demand = rd;
    total_demand -= v;
  }
  return flow;
}

// Implements [Lemma 4.1 (Algorithm 1), TODO: new paper]
// TODO: note, returns the flow scaled up by scale
//
std::tuple<CapacityT, std::vector<CapacityT>, std::vector<bool>>
WeightedPushRelabelOnShortcut(ShortcutGraph sg, std::vector<CapacityT> demand,
                              CapacityT kappa) {
  // Run weighted push relabel on the scaled up instance
  Graph g = sg.shortcut * kappa;
  auto w = sg.weights;
  // TODO: check if the demand is already scaled up?
  demand.resize(g.n);
  for (auto &d : demand) d *= sg.scale;

  CapacityT total_capacity = 0;
  for (auto e : g.Edges()) total_capacity += g.capacity[e];

  WeightT h =
      WeightT(g.n * (6 * sg.L * sg.scale + 100 * kappa * log2(total_capacity)));

  auto [flow_value, flow, residual_demand] =
      WeightedPushRelabel(g, demand, w, h);

  CapacityT total_source = 0, total_sink = 0;
  for (auto d : demand) (d > 0 ? total_source : total_sink) += d;

  if (flow_value == std::min(total_source, total_sink)) {
    // TODO: how should the cut look like here? for not just empty vector
    return {flow_value, flow, {}};
  }

  // Calculate distance layers S[i]

  std::vector<std::list<int>> S(h + 2);
  std::vector<std::pair<int, std::list<int>::iterator>> where(g.n);
  for (auto v : g.Vertices()) {
    where[v] = {h + 1, S[h + 1].emplace(S[h + 1].end(), v)};
  }

  auto Relax = [&](Vertex v, WeightT d) {
    if (d < where[v].first) {
      S[where[v].first].erase(where[v].second);
      where[v] = {d, S[d].emplace(S[d].end(), v)};
    }
  };

  for (auto v : g.Vertices()) {
    if (residual_demand[v] > 0) Relax(v, 0);
  }

  for (int d : std::views::iota(0, h + 1)) {
    for (auto v : S[d]) {
      assert(residual_demand[v] >= 0);
      for (auto e : g.out_edges[v]) {  // forward edges
        if (flow[e] == g.capacity[e]) continue;
        WeightT wf = w[e];
        // shorten forward DAG edges to wf[e] = 0
        if (!sg.IsStarEdge(e) && !sg.IsTopLevelEdge(e) &&
            sg.tau[g.tail[e]] < sg.tau[g.head[e]])
          wf = 0;
        Relax(g.head[e], d + wf);
      }
      for (auto e : g.in_edges[v]) {  // reverse edges
        if (flow[e] == 0) continue;
        Relax(g.tail[e], d + w[e]);
      }
    }
  }

  // Calculate min-capacity layered cut (S[<=i], S[>i])
  std::vector<CapacityT> vol(h + 2), crossing(h + 2);
  for (auto e : g.Edges()) {
    int l = where[g.tail[e]].first;
    int r = where[g.head[e]].first;
    if (sg.IsTopLevelEdge(e)) {
      vol[l] += sg.shortcut.capacity[e];
      vol[r] += sg.shortcut.capacity[e];
    }
    if (r > l) {
      crossing[l] += sg.shortcut.capacity[e];
      crossing[r] -= sg.shortcut.capacity[e];
    }
  }
  partial_sum(vol.begin(), vol.end(), vol.begin());
  partial_sum(crossing.begin(), crossing.end(), crossing.begin());

  // crossing[i] is now c(E(S[<=i], S[>i])) * sg.scale
  // vol[i] is now vol[S[<=i]] * sg.scale

  auto best = std::ranges::min(std::views::iota(0, h), {}, [&](int i) {
    return kappa * crossing[i] - vol[i];
  });
  std::vector<bool> left_of_cut(sg.without_shortcut.n);
  for (int i : std::views::iota(0, best + 1)) {
    for (auto v : S[i])
      if (!sg.IsStarVertex(v)) left_of_cut[v] = true;
  }

  return {flow_value, flow, left_of_cut};
}
