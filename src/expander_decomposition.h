#pragma once

#include <memory>
#include <vector>

#include "graph.h"
#include "shortcut_graph.h"

class Witness {
 public:
  virtual std::vector<CapacityT> Route(
      const std::vector<CapacityT>& demand) = 0;
  virtual ~Witness() = default;
};

std::tuple<std::vector<int>, std::unique_ptr<Witness>, ShortcutGraph>
ExpanderDecomposition(const Graph& g, const std::vector<int>& level,
                      CapacityT scale);
