#pragma once

#include <utility>
#include <vector>

#include "graph.h"
#include "shortcut_graph.h"

std::tuple<std::vector<int>, std::vector<int>, ShortcutGraph>
ExpanderDecomposition(const Graph& g, const std::vector<int>& level,
                      CapacityT scale);
