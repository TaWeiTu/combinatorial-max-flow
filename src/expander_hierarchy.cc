#include "expander_hierarchy.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

#include "flow_util.h"

void FlowUnfolding::AddLevel(ShortcutGraph sg, std::vector<int> expanders) {
  sg_.push_back(sg);
  expanders_.push_back(expanders);
}

namespace {

std::vector<CapacityT> ReRoute(const std::vector<CapacityT>& demand,
                               const ShortcutGraph& sg,
                               const std::vector<int>& expanders) {
  // TODO: Implement re-routing by calling weighted push-relabel.
  return {};
}

}  // namespace

std::vector<CapacityT> FlowUnfolding::Unfold(
    std::vector<CapacityT> flow_on_shortcut) {
  const int L = sg_.size();
  // scale up the flow to control error in congestion incurred by rounding up
  // each time.
  for (auto& v : flow_on_shortcut) v *= L;
  for (int l = sg_.size() - 1; l > 0; --l) {
    assert(std::ssize(flow_on_shortcut) == sg_[l].shortcut.m &&
           "flow_on_shortcut is a flow on the level-l shortcut graph");
    assert(std::ranges::all_of(
        sg_[l].shortcut.Edges(), [&, k = sg_.size() - l](Edge e) {
          return flow_on_shortcut[e] <=
                 (sg_[l].IsStarEdge(e)
                      ? sg_[l].shortcut.capacity[e] * (L + 2 * (k - 1))
                      : sg_[l].shortcut.capacity[e] * L +
                            sg_[l].shortcut.capacity[e] / scale_ * inv_phi_ *
                                (L + 2 * (k - 1)));
        }));
    std::vector<CapacityT> demand(g_.n);
    assert(sg_[l].L == l - 1);
    for (Vertex v : g_.Vertices()) {
      demand[v] +=
          flow_on_shortcut[sg_[l].StarEdge(l - 1, v, ShortcutGraph::kToStar)];
      demand[v] -=
          flow_on_shortcut[sg_[l].StarEdge(l - 1, v, ShortcutGraph::kFromStar)];
    }
    auto reroute = ReRoute(demand, sg_[l - 1], expanders_[l - 1]);
    // auto reroute = witness_[l - 1]->Route(demand);
    assert(std::ssize(reroute) == sg_[l - 1].shortcut.m &&
           "rerouted on lower level shortcut graph");
    // map lower-level star edges as well
    for (Vertex v : g_.Vertices()) {
      for (int i = 0; i < l - 1; ++i) {
        reroute[sg_[l].StarEdge(i, v, ShortcutGraph::kToStar)] +=
            flow_on_shortcut[sg_[l].StarEdge(i, v, ShortcutGraph::kToStar)];
        reroute[sg_[l].StarEdge(i, v, ShortcutGraph::kFromStar)] +=
            flow_on_shortcut[sg_[l].StarEdge(i, v, ShortcutGraph::kFromStar)];
      }
    }
    // also keep flow on original edges
    for (Edge e : g_.Edges()) reroute[e] += flow_on_shortcut[e];
    flow_on_shortcut = reroute;
  }
  assert(std::ssize(flow_on_shortcut) == g_.m);
  return FlowRoundingExact(g_, flow_on_shortcut, L);
}

namespace {

std::pair<std::vector<int>, FlowUnfolding> BuildExpanderHierarchy(Graph g) {
  std::vector<int> level(g.m, 0);
  CapacityT total_capacity =
      std::accumulate(g.capacity.begin(), g.capacity.end(), CapacityT(0));
  // L = ceil(log_{4/3}(c(E))) + 1
  const int L = CapacityT(std::ceil(std::log2(total_capacity) /
                                    std::log2(double(4) / 3))) +
                1;
  // TODO: Check that these parameters are set correctly
  const CapacityT inv_phi = CapacityT(std::pow(std::log2(g.n), 8));
  // This corresopnds to 1/psi in the paper.
  const CapacityT scale = L * 500 * inv_phi;
  FlowUnfolding fu(g, scale, inv_phi);
  while (true) {
    auto [new_level, witness, sg] = ExpanderDecomposition(g, level, scale);
    fu.AddLevel(sg, std::move(witness));
    if (level == new_level) break;
    level = new_level;
  }
  fu.AddLevel(ShortcutGraph(g, level, scale, /*skip_top_level=*/false), {});
  return std::make_pair(level, std::move(fu));
}

}  // namespace

ExpanderHierarchy::ExpanderHierarchy(Graph g) {
  std::tie(level_, fu_) = BuildExpanderHierarchy(g);
}
