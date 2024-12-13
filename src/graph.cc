#include "graph.h"

std::ostream& operator<<(std::ostream& os, const Graph& g) {
  os << g.n << " " << g.m << "\n";
  for (Edge e : g.Edges()) {
    os << g.tail[e] << " -> " << g.head[e] << "\n";
  }
  return os;
}
