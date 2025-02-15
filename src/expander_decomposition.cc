#include "expander_decomposition.h"

#include <array>
#include <memory>
#include <numeric>
#include <ranges>
#include <vector>

#include "cut_matching_game.h"
#include "flow_util.h"
#include "shortcut_graph.h"
#include "weighted_push_relabel.h"

namespace {

class LeafWitness : public Witness {
 public:
  std::vector<CapacityT> Route(const std::vector<CapacityT> &demand) {
    auto flow = PushRelabelOnExpander(expander_, phi_, demand);
  }

 private:
  Graph expander_;
  CapacityT phi_;
};

class InternalWitness : public Witness {
 public:
  InternalWitness(ShortcutGraph sg, Subgraph s1, ShortcutGraph sg1,
                  std::unique_ptr<Witness> w1, Subgraph s2, ShortcutGraph sg2,
                  std::unique_ptr<Witness> w2)
      : sg_(sg),
        subgraphs_({s1, s2}),
        shortcut_subgraphs_({sg1, sg2}),
        child_witness_({std::move(w1), std::move(w2)}) {}
  std::vector<CapacityT> Route(const std::vector<CapacityT> &demand) {
    std::vector<std::vector<CapacityT>> subgraph_demand(2);
    for (int i = 0; i < 2; ++i) subgraph_demand[i].resize(subgraphs_[i].g.n);
    for (Vertex v : sg_.g.Vertices()) {
      int s = -1, i = -1;
      for (int j = 0; j < 2; ++j) {
        if (subgraphs_[j].inv_vertex_map.find(v) !=
            subgraphs_[j].inv_vertex_map.end()) {
          s = j;
          i = subgraphs_[j].inv_vertex_map[v];
        }
      }
      subgraph_demand[s][i] = demand[v];
    }
    assert(std::accumulate(subgraph_demand[0].begin(), subgraph_demand[0].end(),
                           CapacityT(0)) == 0 &&
           std::accumulate(subgraph_demand[1].begin(), subgraph_demand[1].end(),
                           CapacityT(0)) == 0 &&
           "the demand must respect the hierarchy");
    std::vector<CapacityT> flow(sg_.shortcut.m);
    for (int i = 0; i < 2; ++i) {
      auto subgraph_flow = child_witness_[i]->Route(subgraph_demand[i]);
      assert(subgraph_flow.size() == shortcut_subgraphs_[i].shortcut.m &&
             "child_witness_[i]->Route() returns a flow on the shortcut graph "
             "of the subgraph");
      for (Edge e : shortcut_subgraphs_[i].shortcut.Edges()) {
        if (shortcut_subgraphs_[i].edge_map[e] == -1) {
          auto [l, v, d] = shortcut_subgraphs_[i].star_edge_map[e];
          flow[sg_.StarEdge(l, v, d)] += subgraph_flow[e];
        } else {
          flow[subgraphs_[i].edge_map[shortcut_subgraphs_[i].edge_map[e]]] +=
              subgraph_flow[e];
        }
      }
    }
    return flow;
  }

 private:
  ShortcutGraph sg_;
  std::array<std::unique_ptr<Witness>, 2> child_witness_;
  std::array<Subgraph, 2> subgraphs_;
  std::array<ShortcutGraph, 2> shortcut_subgraphs_;
};

class MatchingPlayerImpl : public MatchingPlayer {
 public:
  MatchingPlayerImpl(const ShortcutGraph &sg) : MatchingPlayer(), sg_(sg) {}
  ~MatchingPlayerImpl() = default;
  std::pair<std::vector<bool>,
            std::vector<std::tuple<Vertex, Vertex, CapacityT>>>
  Match(const std::vector<CapacityT> &subdemand,
        const std::vector<bool> &bipartition) {
    std::vector<CapacityT> demand(n_);
    for (Vertex v = 0; v < n_; ++v) {
      demand[v] = bipartition[v] ? -subdemand[v] : subdemand[v];
    }
    auto [value, flow, cut] =
        WeightedPushRelabelOnShortcut(sg_, demand, 50 * phi_);
    auto &m = fd_.emplace_back(sg_.shortcut, flow);
    std::vector<std::tuple<Vertex, Vertex, CapacityT>> matching;
    for (auto [k, v] : m.Demand()) matching.emplace_back(k.first, k.second, v);
    return std::make_pair(cut, matching);
  }

  std::unique_ptr<Witness> ExtractWitness() {
    Graph expander(n_);
    for (const auto &fd : fd_) {
      for (auto [k, v] : fd.Demand()) expander.AddEdge(k.first, k.second, v);
    }
  }

 private:
  int n_;
  ShortcutGraph sg_;
  CapacityT phi_;
  std::vector<FlowDecomposition> fd_;
};

}  // namespace

std::tuple<std::vector<int>, std::unique_ptr<Witness>, ShortcutGraph>
ExpanderDecomposition(const Graph &g, const std::vector<int> &level,
                      CapacityT scale) {
  const CapacityT phi = 100;  // TODO: update phi.
  std::vector<CapacityT> demand(g.n);
  const int max_l = *std::max_element(level.begin(), level.end());
  const auto top_level = std::ranges::to<std::vector<Edge>>(
      g.Edges() |
      std::views::filter([max_l, &level](int e) { return level[e] == max_l; }));
  for (Edge e : top_level) {
    demand[g.tail[e]] += g.capacity[e];
    demand[g.head[e]] += g.capacity[e];
  }
  ShortcutGraph sg(g, level, scale);
  std::unique_ptr<CutMatchingGame> cmg =
      std::make_unique<MockCutMatchingGame>();
  MatchingPlayerImpl matching_player(sg);
  auto result = cmg->Run(g.n, demand, &matching_player);

  std::vector<int> new_level(g.m);

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
    std::vector<std::unique_ptr<Witness>> subgraph_witness(2);
    for (int i = 0; i < 2; ++i) {
      std::vector<Vertex> vtx;
      std::vector<int> subgraph_level;
      for (Vertex v : g.Vertices()) {
        if (cut[v]) {
          vtx.push_back(v);
          subgraph_level.push_back(level[v]);
        }
        cut[v] = !cut[v];
      }
      subgraphs[i] = g.VertexSubgraph(vtx);
      std::vector<int> subgraph_new_level;
      std::tie(subgraph_new_level, subgraph_witness[i], shortcut_subgraphs[i]) =
          ExpanderDecomposition(subgraphs[i].g, subgraph_level, scale);
      for (Edge e : subgraphs[i].g.Edges()) {
        new_level[subgraphs[i].edge_map[e]] = subgraph_new_level[e];
      }
    }

    std::unique_ptr<Witness> witness = std::make_unique<InternalWitness>(
        sg, subgraphs[0], shortcut_subgraphs[0], std::move(subgraph_witness[0]),
        subgraphs[1], shortcut_subgraphs[1], std::move(subgraph_witness[1]));
    return std::make_tuple(new_level, std::move(witness), sg);
  } else {
    // certified that a large portion of demand is expanding
    auto subdemand = std::get<std::vector<CapacityT>>(result);
    // extract a large set of expanding terminal edges
    std::vector<Edge> expanding_top_level;
    for (Edge e : top_level) {
      if (subdemand[g.head[e]] * 2 >= demand[g.head[e]] &&
          subdemand[g.tail[e]] * 2 >= demand[g.tail[e]]) {
        expanding_top_level.push_back(e);
      } else {
        new_level[e] = max_l + 1;
      }
    }
    std::vector<CapacityT> expanding_demand(g.n);
    for (Edge e : expanding_top_level) {
      expanding_demand[g.tail[e]] += g.capacity[e];
      expanding_demand[g.head[e]] += g.capacity[e];
    }
    // re-run the cut-matching game to construct the witness
    MatchingPlayerImpl expanding_matching_player(sg);
    auto expanding_result =
        cmg->Run(g.n, expanding_demand, &expanding_matching_player);
    assert(
        std::holds_alternative<CutMatchingGame::Expanding>(expanding_result) &&
        "The input demand must be expanding");
    return std::make_tuple(
        new_level, std::move(expanding_matching_player.ExtractWitness()), sg);
  }
}
