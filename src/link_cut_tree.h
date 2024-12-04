#pragma once

#include "graph.h"

// Need V + V -> V
// Need U::Apply(U, V) -> V
// Need U::Compose(U, U) -> U
// Need V()
template <typename U, typename V>
class LinkCutTree {
 public:
  virtual void Link(Edge e) = 0;
  virtual void Cut(Edge e) = 0;
  virtual void Update(Vertex u, Vertex v, U update) = 0;
  virtual V Query(Vertex u, Vertex v) = 0;
};
