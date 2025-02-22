#pragma once

#include <cassert>
#include <vector>

#include "link_cut_tree.h"

template <typename U, typename V>
class NaiveLinkCutTree {
 public:
  NaiveLinkCutTree(int n) : values_(n), parent_(n, -1) {}

  void Link(Vertex c, Vertex p, V edge_value) {
    assert(parent_[c] == -1);
    parent_[c] = p;
    values_[c] = edge_value;
  }

  void CutParent(Vertex u) {
    assert(parent_[u] != -1);
    parent_[u] = -1;
    values_[u] = V();
  }

  Vertex GetRoot(Vertex u) {
    for (; parent_[u] != -1; u = parent_[u]);
    return u;
  }

  V QueryParentEdge(Vertex u) { return values_[u]; };

  V QueryPathToRoot(Vertex u) {
    V v;
    for (; parent_[u] != -1; u = parent_[u]) v = v + values_[u];
    return v;
  }

  void UpdatePathToRoot(Vertex u, U update) {
    for (; parent_[u] != -1; u = parent_[u])
      values_[u] = U::Apply(update, values_[u]);
  }

 private:
  std::vector<V> values_;
  std::vector<Vertex> parent_;
};
