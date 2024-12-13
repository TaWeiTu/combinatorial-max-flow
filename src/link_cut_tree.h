#pragma once

#include <cassert>
#include <memory>
#include <vector>

#include "splay.h"
#include "types.h"

/**
 * Keeps track of a dynamic rooted tree.
 * Allows for updates and aggregates on (u -> root) paths
 *
 * Need V + V -> V
 * Need U::Apply(U, V) -> V
 * Need U::Compose(U, U) -> U
 * Need V() and U()
 **/

template <typename U, typename V>
class LinkCutTree {
 public:
  LinkCutTree(int n) : nodes_(n) {
    for (int i = 0; i < n; ++i) {
      nodes_[i] = std::make_unique<splay::SplayNode<U, V>>(i);
    }
  }

  virtual void Link(Vertex child, Vertex parent, V edge_value) {
    nodes_[child]->Access();
    nodes_[parent]->Access();
    assert(!nodes_[child]->pfa && !nodes_[child]->fa);
    nodes_[child]->pfa = nodes_[parent].get();
    nodes_[child]->value = edge_value;
    nodes_[child]->Pull();
  }

  virtual void CutParent(Vertex u) {
    nodes_[u]->Access();
    assert(nodes_[u]->child[0]);
    nodes_[u]->value = V();
    nodes_[u]->child[0]->fa = nullptr;
    nodes_[u]->child[0] = nullptr;
    nodes_[u]->Pull();
  }

  virtual Vertex GetRoot(Vertex u) {
    auto p = nodes_[u].get();
    p->Access();
    p->Splay();
    while (p->child[0]) p = p->child[0];
    p->Splay();
    return p->id;
  }

  virtual V QueryParentEdge(Vertex u) {
    nodes_[u]->Splay();
    return nodes_[u]->value;
  }

  virtual V QueryPathToRoot(Vertex u) {
    nodes_[u]->Access();
    return nodes_[u]->aggregation;
  }

  virtual void UpdatePathToRoot(Vertex u, U update) {
    nodes_[u]->Access();
    nodes_[u]->Update(update);
  }

 private:
  std::vector<std::unique_ptr<splay::SplayNode<U, V>>> nodes_;
};
