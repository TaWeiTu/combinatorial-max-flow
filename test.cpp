#include <iostream>

#include "naive_link_cut_tree.h"

int main() {
  using V = int;

  struct U {
    V x;
    U(V x) : x(x) {}
    static V Apply(U a, V b) { return a.x + b; }
    static U Compose(U a, U b) { return U(a.x + b.x); }
  };

  std::vector<std::pair<Vertex, Vertex>> edges;
  edges.emplace_back(0, 1);
  edges.emplace_back(1, 2);
  edges.emplace_back(0, 2);
  LinkCutTree<U, V>* t = new NaiveLinkCutTree<U, V>(3, edges);
  t->Link(0);
  t->Link(1);
  t->Update(0, 2, U(1));
  t->Cut(1);
  t->Link(2);
  t->Update(0, 2, U(2));
  std::cout << t->Query(0, 2) << "\n";
  std::cout << t->Query(0, 1) << "\n";
  std::cout << t->Query(1, 2) << "\n";
  delete t;
}
