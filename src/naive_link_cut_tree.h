#pragma once

#include <cassert>
#include <vector>

#include "link_cut_tree.h"

template <typename U, typename V>
class NaiveLinkCutTree : public LinkCutTree<U, V> {
 public:
  NaiveLinkCutTree(int n) : values_(n), parent_(n, -1) {}

  void Link(Vertex c, Vertex p) {
    assert(parent_[c] == -1);
    parent_[c] = p;
  }

  void CutParent(Vertex u) {
    assert(parent_[u] != -1);
    parent_[u] = -1;
    SetParentEdge(u, V());
  }

  Vertex GetRoot(Vertex u) {
    for (; parent_[u] != -1; u = parent_[u]);
    return u;
  }

  void SetParentEdge(Vertex u, V value) { values_[u] = value; };

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
