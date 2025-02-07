#pragma once

#include <vector>

#include "graph.h"

class ShortcutGraph {
 private:
  Graph g_;
  std::vector<int> levels_;

 public:
  ShortcutGraph(const Graph &g, const std::vector<int> &levels);
  std::pair<Graph, std::vector<WeightT>> ToGraphAndWeights(
      CapacityT scale) const;
};
