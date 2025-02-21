#pragma once

#include "graph.h"
#include "shortcut_graph.h"

class ExpanderHierarchy {
 public:
  ExpanderHierarchy(const Graph& g);
  std::vector<CapacityT> UnwrapFlow(
      const std::vector<CapacityT>& flow_on_shortcut_graph);

  ShortcutGraph shortcut_graph_;
};
