#include "expander_decomposition.h"

#include <algorithm>
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
  LeafWitness(Graph expander, CapacityT phi, ShortcutGraph sg,
              std::vector<int> expander_edge_map,
              std::vector<FlowDecomposition> fd)
      : Witness(),
        expander_(expander),
        phi_(phi),
        sg_(sg),
        expander_edge_map_(expander_edge_map),
        fd_(fd) {}
  std::vector<CapacityT> Route(const std::vector<CapacityT> &demand) {
    auto flow_on_expander = PushRelabelOnExpander(expander_, phi_, demand);
    const int R = fd_.size();
    std::vector<MultiCommodityDemand> demand_per_round(R);
    for (Edge e : expander_.Edges()) {
      int r = expander_edge_map_[e];
      demand_per_round[r][std::make_pair(
          expander_.tail[e], expander_.head[e])] += flow_on_expander[e];
    }
    std::vector<CapacityT> flow(sg_.shortcut.m);
    for (int r = 0; r < R; ++r) {
      auto f = fd_[r].Route(demand_per_round[r]);
      assert(std::ssize(f) == sg_.shortcut.m &&
             "the flow is on the shortcut graph");
      for (Edge e : sg_.shortcut.Edges()) flow[e] += f[e];
    }
    return flow;
  }

 private:
  Graph expander_;
  CapacityT phi_;
  ShortcutGraph sg_;
  // map each edge in the expander into the round number (of the cut-matching
  // game) when it was added to the expander.
  std::vector<int> expander_edge_map_;
  std::vector<FlowDecomposition> fd_;
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
    for (Vertex v : sg_.without_shortcut.Vertices()) {
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
      assert(std::ssize(subgraph_flow) == shortcut_subgraphs_[i].shortcut.m &&
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
  std::array<Subgraph, 2> subgraphs_;
  std::array<ShortcutGraph, 2> shortcut_subgraphs_;
  std::array<std::unique_ptr<Witness>, 2> child_witness_;
};

class MatchingPlayerImpl : public MatchingPlayer {
 public:
  MatchingPlayerImpl(const ShortcutGraph &sg, CapacityT phi)
      : MatchingPlayer(), sg_(sg), phi_(phi) {}
  ~MatchingPlayerImpl() = default;
  std::pair<std::vector<bool>,
            std::vector<std::tuple<Vertex, Vertex, CapacityT>>>
  Match(const std::vector<CapacityT> &subdemand,
        const std::vector<bool> &bipartition) {
    const int n = sg_.without_shortcut.n;
    std::vector<CapacityT> demand(n);
    for (Vertex v = 0; v < n; ++v) {
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
    Graph expander(sg_.without_shortcut.n);
    std::vector<int> expander_edge_map;
    for (int r = 0; r < std::ssize(fd_); ++r) {
      for (auto [k, v] : fd_[r].Demand()) {
        expander.AddEdge(k.first, k.second, v);
        expander_edge_map.push_back(r);
      }
    }
    return std::make_unique<LeafWitness>(expander, phi_, sg_, expander_edge_map,
                                         fd_);
  }

 private:
  ShortcutGraph sg_;
  CapacityT phi_;
  std::vector<FlowDecomposition> fd_;
};

}  // namespace

std::tuple<std::vector<int>, std::unique_ptr<Witness>, ShortcutGraph>
ExpanderDecomposition(const Graph &g, const std::vector<int> &level,
                      CapacityT scale) {
  std::vector<CapacityT> demand(g.n);
  const int max_l = *std::max_element(level.begin(), level.end());
  std::vector<Edge> top_level;
  for (auto e : g.Edges())
    if (level[e] == max_l) top_level.push_back(e);
  for (Edge e : top_level) {
    demand[g.tail[e]] += g.capacity[e];
    demand[g.head[e]] += g.capacity[e];
  }
  ShortcutGraph sg(g, level, scale);
  const CapacityT phi = 100;  // TODO: figuer out the value of phi.
  MatchingPlayerImpl matching_player(sg, phi);
  auto result = CutMatchingGame(demand, &matching_player);

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

    // re-run the cut-matching game to construct the witness
    const CapacityT new_phi = 0;  // TODO: figure out the value of new_phi.
    MatchingPlayerImpl expanding_matching_player(sg, new_phi);
    auto expanding_result =
        CutMatchingGame(expanding_demand, &expanding_matching_player);
    assert(std::holds_alternative<Expanding>(expanding_result) &&
           "The input demand must be expanding");
    return std::make_tuple(
        new_level, std::move(expanding_matching_player.ExtractWitness()), sg);
  }
}
