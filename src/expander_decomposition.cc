#include "expander_decomposition.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <ranges>
#include <vector>

#include "cut_matching_game.h"
#include "flow_util.h"
#include "shortcut_graph.h"
#include "weighted_push_relabel.h"

namespace {

class MatchingPlayerImpl : public MatchingPlayer {
 public:
  MatchingPlayerImpl(const ShortcutGraph &sg, CapacityT phi)
      : MatchingPlayer(), sg_({sg, sg.Reverse()}), phi_(phi) {}
  ~MatchingPlayerImpl() = default;
  std::pair<
      std::array<std::pair<std::vector<bool>,
                           std::vector<std::tuple<Vertex, Vertex, CapacityT>>>,
                 2>,
      CapacityT>
  Match(const std::vector<CapacityT> &subdemand,
        const std::vector<bool> &bipartition) {
    const int n = sg_[0].without_shortcut.n;
    std::vector<CapacityT> demand(n);
    for (Vertex v = 0; v < n; ++v) {
      demand[v] = bipartition[v] ? -subdemand[v] : subdemand[v];
    }
    auto &fd = fd_.emplace_back();
    std::array<std::pair<std::vector<bool>,
                         std::vector<std::tuple<Vertex, Vertex, CapacityT>>>,
               2>
        cut_and_matching;
    for (int rev = 0; rev < 2; ++rev) {
      auto [value, flow, c] =
          WeightedPushRelabelOnShortcut(sg_[rev], demand, 50 * phi_);
      cut_and_matching[rev].first = c;
      // TODO: Only include flow paths that are short.
      fd[rev] = FlowDecomposition(sg_[rev].shortcut, flow);
      for (auto [k, v] : fd[rev].Demand()) {
        auto tail = k.first, head = k.second;
        if (rev) std::swap(tail, head);
        cut_and_matching[rev].second.emplace_back(tail, head, v);
      }
    }
    return std::make_pair(cut_and_matching, sg_[0].scale);
  }

 private:
  std::array<ShortcutGraph, 2> sg_;
  CapacityT phi_;
  std::vector<std::array<FlowDecomposition, 2>> fd_;
};

}  // namespace

std::tuple<std::vector<int>, std::vector<int>, ShortcutGraph>
ExpanderDecomposition(const Graph &g, const std::vector<int> &level,
                      CapacityT scale) {
  ShortcutGraph sg(g, level, scale, /*skip_top_level=*/true);
  if (g.n == 1 || g.m == 0) {
    return std::make_tuple(std::vector<int>{}, std::vector<int>(g.n, 0), sg);
  }

  std::vector<CapacityT> demand(g.n);
  const int max_l = std::ranges::max(level);
  std::vector<Edge> top_level;
  for (auto e : g.Edges())
    if (level[e] == max_l) top_level.push_back(e);
  for (Edge e : top_level) {
    demand[g.tail[e]] += g.capacity[e];
    demand[g.head[e]] += g.capacity[e];
  }
  // TODO: figure out the value of phi.
  const CapacityT inv_phi = CapacityT(std::pow(std::log2(g.n), 3));
  MatchingPlayerImpl matching_player(sg, inv_phi);
  auto result = CutMatchingGame(demand, &matching_player);

  std::vector<int> new_level = level;

  if (std::holds_alternative<std::vector<bool>>(result)) {
    // found a balanced sparse cut: recurse on both sides
    auto cut = std::get<std::vector<bool>>(result);
    std::vector<std::vector<Edge>> cut_edges(2);
    std::vector<CapacityT> cut_capacity(2);
    for (Edge e : g.Edges()) {
      if (cut[g.head[e]] != cut[g.tail[e]]) {
        cut_edges[cut[g.head[e]]].push_back(e);
        cut_capacity[cut[g.head[e]]] += g.capacity[e];
      }
    }
    int idx = cut_capacity[0] < cut_capacity[1] ? 0 : 1;
    for (Edge e : cut_edges[idx]) new_level[e] = max_l + 1;
    std::vector<Subgraph> subgraphs(2);
    std::vector<ShortcutGraph> shortcut_subgraphs(2);
    std::vector<int> expanders(g.n);
    int last = 0;
    for (int i = 0; i < 2; ++i) {
      std::vector<Vertex> vtx;
      for (Vertex v : g.Vertices()) {
        if (cut[v] == i) vtx.push_back(v);
      }
      subgraphs[i] = g.VertexSubgraph(vtx);
      std::vector<int> subgraph_level(subgraphs[i].g.m);
      for (Edge e : subgraphs[i].g.Edges()) {
        subgraph_level[e] = level[subgraphs[i].inv_edge_map[e]];
      }
      std::vector<int> subgraph_new_level, subgraph_expanders;
      std::tie(subgraph_new_level, subgraph_expanders, shortcut_subgraphs[i]) =
          ExpanderDecomposition(subgraphs[i].g, subgraph_level, scale);
      for (Edge e : subgraphs[i].g.Edges()) {
        new_level[subgraphs[i].edge_map[e]] = subgraph_new_level[e];
      }
      for (Vertex v : vtx) {
        expanders[v] =
            last + subgraph_expanders[subgraphs[i].inv_vertex_map[v]];
      }
      last += std::ranges::max(subgraph_expanders);
    }

    return std::make_tuple(new_level, expanders, sg);
  } else {
    if (std::holds_alternative<Expanding>(result)) {
      // certified that everything was expanding: return the witness
      return std::make_tuple(level, std::vector<int>(g.n, 0), sg);
    }

    // certified that a large portion of demand is expanding:
    //  * we extract this expanding demand, and rerun the cmg on it to build a
    //  witness.
    //  * we promote the non-expanding part (small fraction) to the next level.
    auto subdemand = std::get<std::vector<CapacityT>>(result);
    assert(
        std::accumulate(subdemand.begin(), subdemand.end(), CapacityT(0)) * 8 >=
            std::accumulate(demand.begin(), demand.end(), CapacityT(0)) * 7 &&
        "the cut-matching game certifies 7/8 fraction of the demand is "
        "expanding.");
    // extract a large set of expanding terminal edges
    std::vector<Edge> expanding_top_level;
    CapacityT total_expanding_capacity = 0, total_capacity = 0;
    for (Edge e : top_level) {
      total_capacity += g.capacity[e];
      if (subdemand[g.head[e]] * 2 >= demand[g.head[e]] &&
          subdemand[g.tail[e]] * 2 >= demand[g.tail[e]]) {
        expanding_top_level.push_back(e);
        total_expanding_capacity += g.capacity[e];
      } else {
        new_level[e] = max_l + 1;
      }
    }
    assert(total_expanding_capacity * 2 >= total_capacity &&
           "half of the total capacity is expanding");
    std::vector<CapacityT> expanding_demand(g.n);
    for (Edge e : expanding_top_level) {
      expanding_demand[g.tail[e]] += g.capacity[e];
      expanding_demand[g.head[e]] += g.capacity[e];
    }
    assert(std::ranges::all_of(std::ranges::iota_view(0, g.n),
                               [&](int i) {
                                 return expanding_demand[i] <= subdemand[i] * 2;
                               }) &&
           "the expanding demand should be entry-wise bounded by the output of "
           "the cut-matching game up to a factor of 2");

    return std::make_tuple(new_level, std::vector<int>(g.n, 0), sg);
  }
}
