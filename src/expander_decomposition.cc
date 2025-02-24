#include "expander_decomposition.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numeric>
#include <ranges>
#include <vector>

#include "cut_matching_game.h"
#include "flow_util.h"
#include "shortcut_graph.h"
#include "weighted_push_relabel.h"

namespace {

class EmptyWitness : public Witness {  // for an empty graph
 public:
  EmptyWitness(int n, int m) : n_(n), m_(m) {}
  virtual ~EmptyWitness() = default;
  std::vector<CapacityT> Route(const std::vector<CapacityT> &demand) {
    assert(std::ssize(demand) == n_ &&
           std::ranges::all_of(demand, [&](auto v) { return v == 0; }));
    return std::vector<CapacityT>(m_, 0);
  }

 private:
  int n_, m_;
};

class LeafWitness : public Witness {
 public:
  LeafWitness(Graph expander, CapacityT phi, CapacityT kappa, ShortcutGraph sg,
              std::vector<std::pair<int, int>> expander_edge_map,
              std::vector<std::array<FlowDecomposition, 2>> fd)
      : Witness(),
        expander_(expander),
        scale_(sg.scale),
        phi_(phi),
        kappa_(kappa),
        sg_(sg),
        expander_edge_map_(expander_edge_map),
        fd_(fd) {}
  virtual ~LeafWitness() = default;
  std::vector<CapacityT> Route(const std::vector<CapacityT> &demand) {
    // To make things integral, route the scaled up demand first instead.
    auto scaled_up_demand = demand;
    for (auto &v : scaled_up_demand) v *= scale_;
    std::cerr << "scaled_up_demand = ";
    for (auto &v : scaled_up_demand) std::cerr << v << " ";
    std::cerr << "\n";
    std::cerr << "witness = \n";
    for (Edge e : expander_.Edges()) {
      auto [x, y, c] = expander_.EdgeInfo(e);
      std::cerr << x << " -> " << y << ": " << c << "\n";
    }
    auto flow_on_expander =
        PushRelabelOnExpander(expander_, phi_, scaled_up_demand);
    std::cerr << "done push-relabel on expander\n";
    assert(FlowToDemand(expander_, flow_on_expander) == scaled_up_demand);
    assert(std::ranges::all_of(expander_.Edges(), [&](Edge e) {
      // TODO: this 10 should be an O(log n) term.
      return flow_on_expander[e] <= expander_.capacity[e] * 50 * phi_ * 10;
    }));
    // If this is the k-th level in the unfolding process, then
    // flow_on_expander[e] should be bounded by c[e] * k/psi * O(log n / phi)
    // for each expander edge e.
    const int R = fd_.size();
    std::vector<std::array<MultiCommodityDemand, 2>> demand_per_round(R);
    for (Edge e : expander_.Edges()) {
      auto [r, rev] = expander_edge_map_[e];
      auto tail = expander_.tail[e], head = expander_.head[e];
      if (rev) std::swap(tail, head);
      demand_per_round[r][rev][std::make_pair(tail, head)] +=
          flow_on_expander[e];
    }
    std::vector<CapacityT> flow(sg_.shortcut.m);
    for (int r = 0; r < R; ++r) {
      for (int rev = 0; rev < 2; ++rev) {
        auto f = fd_[r][rev].Route(demand_per_round[r][rev]);
        assert(std::ssize(f) == sg_.shortcut.m &&
               "the flow is on the shortcut graph");
        for (Edge e : sg_.shortcut.Edges()) flow[e] += f[e];
      }
    }
    std::cerr << "flow = ";
    for (auto &v : flow) std::cerr << v << " ";
    std::cerr << "\n";
    // Now, scale the flow back down.
    return FlowRoundingExact(sg_.shortcut, flow, scale_);
  }

 private:
  // expander_ is a phi_-expander, where each edge is part of a matching
  // embeddable into the original graph with congestion kappa_.
  Graph expander_;
  CapacityT scale_, phi_, kappa_;
  ShortcutGraph sg_;
  // map each edge in the expander into the round number (of the cut-matching
  // game) when it was added to the expander.
  std::vector<std::pair<int, int>> expander_edge_map_;
  std::vector<std::array<FlowDecomposition, 2>> fd_;
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
  virtual ~InternalWitness() = default;
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
      : MatchingPlayer(), sg_({sg, sg.Reverse()}), phi_(phi) {}
  ~MatchingPlayerImpl() = default;
  std::tuple<std::vector<bool>,
             std::array<std::vector<std::tuple<Vertex, Vertex, CapacityT>>, 2>,
             CapacityT>
  Match(const std::vector<CapacityT> &subdemand,
        const std::vector<bool> &bipartition) {
    const int n = sg_[0].without_shortcut.n;
    std::vector<CapacityT> demand(n);
    for (Vertex v = 0; v < n; ++v) {
      demand[v] = bipartition[v] ? -subdemand[v] : subdemand[v];
    }
    auto &fd = fd_.emplace_back();
    std::vector<bool> cut(n);
    std::array<std::vector<std::tuple<Vertex, Vertex, CapacityT>>, 2> matching;
    for (int rev = 0; rev < 2; ++rev) {
      auto [value, flow, c] =
          WeightedPushRelabelOnShortcut(sg_[rev], demand, 50 * phi_);
      for (Vertex v = 0; v < n; ++v) cut[v] = cut[v] || c[v];
      // cut[v] = cut[v] || (!bipartition[v] && c[v]);
      // TODO: now we (incorrectly) return the actual cut, and not just the cut
      // on the subset on the source side. This is so that the CMG can return
      // forward the cut (if it just had the cut restricted to the source side,
      // that cut might no longer be sparse (and let to infinite recursion in
      // the expander hierarchy)). One potential problem right now is that if we
      // find a out-sparse and in-sparse cut seperately, and then return the
      // union cut it is neither out- nor in-sparse. Likely we need to return
      // the two cuts seperately...
      fd[rev] = FlowDecomposition(sg_[rev].shortcut, flow);
      for (auto [k, v] : fd[rev].Demand()) {
        auto tail = k.first, head = k.second;
        if (rev) std::swap(tail, head);
        matching[rev].emplace_back(tail, head, v);
      }
    }
    return std::make_tuple(cut, matching, sg_[0].scale);
  }

  std::unique_ptr<Witness> ExtractWitness() {
    Graph expander(sg_[0].without_shortcut.n);
    std::vector<std::pair<int, int>> expander_edge_map;
    for (int r = 0; r < std::ssize(fd_); ++r) {
      for (int rev = 0; rev < 2; ++rev) {
        for (auto [k, v] : fd_[r][rev].Demand()) {
          auto tail = k.first, head = k.second;
          if (rev) std::swap(tail, head);
          expander.AddEdge(tail, head, v);
          expander_edge_map.emplace_back(r, rev);
        }
      }
    }
    // TODO: figure out the exact expansion of the pure expander.
    const CapacityT inv_phi =
        CapacityT(std::pow(std::log2(sg_[0].without_shortcut.n), 2));
    return std::make_unique<LeafWitness>(expander, inv_phi, 50 * phi_, sg_[0],
                                         expander_edge_map, fd_);
  }

 private:
  std::array<ShortcutGraph, 2> sg_;
  CapacityT phi_;
  std::vector<std::array<FlowDecomposition, 2>> fd_;
};

}  // namespace

std::tuple<std::vector<int>, std::unique_ptr<Witness>, ShortcutGraph>
ExpanderDecomposition(const Graph &g, const std::vector<int> &level,
                      CapacityT scale) {
  ShortcutGraph sg(g, level, scale, /*skip_top_level=*/true);
  if (g.n == 1 || g.m == 0) {
    return std::make_tuple(std::vector<int>{},
                           std::make_unique<EmptyWitness>(g.n, g.m), sg);
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
    std::vector<std::unique_ptr<Witness>> subgraph_witness(2);
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
    if (std::holds_alternative<Expanding>(result)) {
      // certified that everything was expanding: return the witness
      return std::make_tuple(level, matching_player.ExtractWitness(), sg);
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

    // re-run the cut-matching game to construct the witness
    // TODO: figure out the value of new_phi.
    const CapacityT new_inv_phi = CapacityT(std::pow(std::log2(g.n), 5));
    MatchingPlayerImpl expanding_matching_player(sg, new_inv_phi);
    auto expanding_result =
        CutMatchingGame(expanding_demand, &expanding_matching_player);
    assert(std::holds_alternative<Expanding>(expanding_result) &&
           "The input demand must be expanding");
    return std::make_tuple(new_level,
                           expanding_matching_player.ExtractWitness(), sg);
  }
}
