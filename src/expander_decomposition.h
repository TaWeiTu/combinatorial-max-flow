#pragma once

#include <memory>
#include <vector>

#include "graph.h"

class Witness {
 public:
  virtual std::vector<CapacityT> Route(
      const std::vector<CapacityT>& demand) = 0;
};

std::pair<std::vector<int>, std::unique_ptr<Witness>> ExpanderDecomposition(
    Graph g, const std::vector<int>& level);
