#pragma once

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
  virtual void Link(Vertex child, Vertex parent) = 0;
  virtual void CutParent(Vertex u) = 0;
  virtual Vertex GetRoot(Vertex u) = 0;
  virtual void SetParentEdge(Vertex u, V value) = 0;
  virtual V QueryParentEdge(Vertex u) = 0;
  virtual V QueryPathToRoot(Vertex u) = 0;
  virtual void UpdatePathToRoot(Vertex u, U update) = 0;
};
