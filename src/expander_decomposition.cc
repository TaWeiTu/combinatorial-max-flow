#include "expander_decomposition.h"

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
  std::vector<CapacityT> Route(const std::vector<CapacityT> &demand) {}
};

class InternalWitness : public Witness {
 public:
  std::vector<CapacityT> Route(const std::vector<CapacityT> &demand) {
    std::vector<std::vector<CapacityT>> subgraph_demand(2);
    for (Vertex v : g_.Vertices()) {
      auto [s, i] = inv_vertex_map_[v];
      if (i >= subgraph_demand[s].size()) subgraph_demand[s].resize(i + 1);
      subgraph_demand[inv_vertex_map_[v].second][i] = demand[v];
    }
    assert(std::accumulate(subgraph_demand[0].begin(), subgraph_demand[0].end(),
                           CapacityT(0)) == 0 &&
           std::accumulate(subgraph_demand[1].begin(), subgraph_demand[1].end(),
                           CapacityT(0)) == 0 &&
           "the demand must respect the hierarchy");
    std::vector<CapacityT> flow(g_.m);
    for (int i = 0; i < 2; ++i) {
      auto subgraph_flow = child_witness_[i]->Route(subgraph_demand[i]);
      for (int j = 0; j < subgraph_flow.size(); ++j) {
        flow[edge_map_[i][j]] += subgraph_flow[j];
      }
    }
    return flow;
  }

 private:
  Graph g_;
  std::unique_ptr<Witness> child_witness_[2];
  // map from each vertex/edge in the subgraph to the corresponding vertex/edge
  // in the original graph.
  std::vector<Vertex> vertex_map_[2];
  std::vector<std::pair<Vertex, bool>> inv_vertex_map_;
  std::vector<Edge> edge_map_[2];
  std::vector<std::pair<Edge, bool>> inv_edge_map_;
};

class MatchingPlayerImpl : public MatchingPlayer {
 public:
  MatchingPlayerImpl(const Graph &g, const std::vector<int> &level)
      : MatchingPlayer(), g_(g, level) {}
  ~MatchingPlayerImpl() = default;
  std::pair<std::vector<bool>,
            std::vector<std::tuple<Vertex, Vertex, CapacityT>>>
  Match(const std::vector<CapacityT> &subdemand,
        const std::vector<bool> &bipartition) {
    std::vector<CapacityT> demand(n_);
    for (Vertex v = 0; v < n_; ++v) {
      demand[v] = bipartition[v] ? -subdemand[v] : subdemand[v];
    }
    auto [g_a, value, flow, cut] =
        WeightedPushRelabelOnShortcut(g_, demand, 50 * phi_, scale_);
    auto &m = fd_.emplace_back(g_a, flow);
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
  ShortcutGraph g_;
  CapacityT scale_;
  CapacityT phi_;
  std::vector<FlowDecomposition> fd_;
};

}  // namespace

std::pair<std::vector<int>, std::unique_ptr<Witness>> ExpanderDecomposition(
    const Graph &g, const std::vector<int> &level) {
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
  std::unique_ptr<CutMatchingGame> cmg =
      std::make_unique<MockCutMatchingGame>();
  MatchingPlayerImpl matching_player(g, level);
  auto result = cmg->Run(g.n, demand, &matching_player);

  std::vector<int> new_level(g.m);

  if (std::holds_alternative<std::vector<bool>>(result)) {
    // found a balanced sparse cut: recurse on both sides
    const auto &cut = std::get<std::vector<bool>>(result);
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
    // TODO: Recurse on both sides and construct the witness
    std::unique_ptr<Witness> witness = std::make_unique<InternalWitness>();
    return std::make_pair(new_level, std::move(witness));
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
    MatchingPlayerImpl expanding_matching_player(g, level);
    auto expanding_result =
        cmg->Run(g.n, expanding_demand, &expanding_matching_player);
    assert(
        std::holds_alternative<CutMatchingGame::Expanding>(expanding_result) &&
        "The input demand must be expanding");
    return std::make_pair(
        new_level, std::move(expanding_matching_player.ExtractWitness()));
  }
}
