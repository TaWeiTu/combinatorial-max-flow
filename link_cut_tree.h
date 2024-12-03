#pragma once

// Need V + V -> V
// Need U::Apply(V) -> V
// Need U::Combine(U) -> U
// Need V()
template <typename U, typename V>
struct LinkCutTree {
  using Edge = int;
  using Vertex = int;
  void Link(Vertex u, Vertex v, Edge e);
  void Cut(Vertex u, Vertex v);
  void Update(Vertex u, Vertex v, U t);
  V Query(Vertex u, Vertex v);
  Edge Parent(Vertex u);

  LinkCutTree(int n, int m);
};
