#pragma once

#include <vector>

#include "expander_decomposition.h"
#include "graph.h"

class FlowUnfolding {
 public:
  FlowUnfolding(Graph g, CapacityT scale, CapacityT inv_phi)
      : g_(g), scale_(scale), inv_phi_(inv_phi) {}
  void AddLevel(ShortcutGraph sg, std::unique_ptr<Witness> witness);

  // Unfold the flow on the shortcut graph back to the original graph.
  // NOTE: the flow_on_shortcut is already scaled up by a factor of 1/psi.
  std::vector<CapacityT> Unfold(std::vector<CapacityT> flow_on_shortcut);

 private:
  std::vector<std::unique_ptr<Witness>> witness_;
  std::vector<ShortcutGraph> sg_;
  Graph g_;
  CapacityT scale_, inv_phi_;
};

std::pair<std::vector<int>, FlowUnfolding> BuildExpanderHierarchy(Graph g);
