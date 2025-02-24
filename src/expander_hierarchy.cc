#include "expander_hierarchy.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

void FlowUnfolding::AddLevel(ShortcutGraph sg,
                             std::unique_ptr<Witness> witness) {
  sg_.push_back(sg);
  witness_.push_back(std::move(witness));
}

std::vector<CapacityT> FlowUnfolding::Unfold(
    std::vector<CapacityT> flow_on_shortcut) {
  for (int l = sg_.size() - 1; l > 0; --l) {
    assert(std::ssize(flow_on_shortcut) == sg_[l].shortcut.m &&
           "flow_on_shortcut is a flow on the level-l shortcut graph");
    assert(std::ranges::all_of(
               sg_[l].shortcut.Edges(),
               [&, k = sg_.size() - l](Edge e) {
                 return flow_on_shortcut[e] <=
                        (sg_[l].IsStarEdge(e)
                             ? sg_[l].shortcut.capacity[e] * k
                             : sg_[l].shortcut.capacity[e] / scale_ *
                                   (scale_ + inv_phi_ * k * (k - 1) / 2));
               }) &&
           "the capacity blows on by a factor of (1+1/L) each time");
    std::vector<CapacityT> demand(g_.n);
    assert(sg_[l].L == l);
    for (Vertex v : g_.Vertices()) {
      demand[v] +=
          flow_on_shortcut[sg_[l].StarEdge(l, v, ShortcutGraph::kToStar)];
      demand[v] -=
          flow_on_shortcut[sg_[l].StarEdge(l, v, ShortcutGraph::kFromStar)];
    }
    auto reroute = witness_[l]->Route(demand);
    assert(std::ssize(reroute) == sg_[l - 1].shortcut.m &&
           "rerouted on lower level shortcut graph");
    assert(std::ranges::all_of(
        sg_[l - 1].shortcut.Edges(), [&, k = sg_.size() - 1](Edge e) {
          return reroute[e] <= (sg_[l - 1].IsStarEdge(e)
                                    ? sg_[l - 1].shortcut.capacity[e]
                                    : sg_[l - 1].shortcut.capacity[e] / scale_ *
                                          (scale_ + inv_phi_ * k));
        }));
    flow_on_shortcut = reroute;
  }

  // flow should now be a multiple of scale_, so it is safe to scale down
  for (auto &f : flow_on_shortcut) {
    assert(f % scale_ == 0);
    f /= scale_;
  }
  return flow_on_shortcut;
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
  // TODO: Check these values.
  const CapacityT inv_phi = CapacityT(std::pow(std::log2(g.n), 10));
  // This corresopnds to 1/psi in the paper.
  // Note that in the paper psi is set to phi/L but here we set it to psi/L^2.
  // This is so that we can stay integral throughout the implementation.
  const CapacityT scale = L * L * inv_phi;
  FlowUnfolding fu(g, scale, inv_phi);
  while (true) {
    auto [new_level, witness, sg] = ExpanderDecomposition(g, level, scale);
    fu.AddLevel(sg, std::move(witness));
    if (level == new_level) break;
    level = new_level;
  }
  return std::make_pair(level, std::move(fu));
}

}  // namespace

ExpanderHierarchy::ExpanderHierarchy(Graph g) {
  std::tie(level_, fu_) = BuildExpanderHierarchy(g);
}
