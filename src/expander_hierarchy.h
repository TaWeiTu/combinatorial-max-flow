#pragma once

#include <vector>

#include "expander_decomposition.h"
#include "graph.h"

class FlowUnfolding {
 public:
  FlowUnfolding() = default;
  FlowUnfolding(Graph g, CapacityT scale, CapacityT inv_phi)
      : g_(g), scale_(scale), inv_phi_(inv_phi) {}
  void AddLevel(ShortcutGraph sg, std::vector<int> expanders);

  // Unfold the flow on the shortcut graph back to the original graph.
  // NOTE: the flow_on_shortcut is already scaled up by a factor of 1/psi.
  std::vector<CapacityT> Unfold(std::vector<CapacityT> flow_on_shortcut);
  ShortcutGraph GetShortcutGraph() const { return sg_.back(); }

  CapacityT Scale() const { return scale_; }

 private:
  std::vector<std::vector<int>> expanders_;
  std::vector<ShortcutGraph> sg_;
  Graph g_;
  CapacityT scale_, inv_phi_;
};

class ExpanderHierarchy {
  std::vector<int> level_;
  FlowUnfolding fu_;

 public:
  ExpanderHierarchy(Graph g);

  const std::vector<int>& GetLevel() const { return level_; }
  ShortcutGraph GetShortcutGraph() const { return fu_.GetShortcutGraph(); }
  std::vector<CapacityT> Unfold(std::vector<CapacityT> flow) {
    return fu_.Unfold(flow);
  }

  CapacityT Scale() const { return fu_.Scale(); }
};
