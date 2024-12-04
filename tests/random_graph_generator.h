#pragma once

#include <catch2/catch_all.hpp>
#include <random>

#include "../src/graph.h"

namespace {
using namespace std;
class RandomGraphGenerator final : public Catch::Generators::IGenerator<Graph> {
  std::mt19937_64 rng;
  int nodes, edges;
  int test_count = 0;
  Graph g;

 public:
  RandomGraphGenerator(int nodes = -1, int edges = -1)
      : rng(Catch::rngSeed()), nodes(nodes), edges(edges) {
    static_cast<void>(next());
  }
  Graph const &get() const override { return g; }
  bool next() override {
    ++test_count;
    int n = nodes != -1 ? nodes : rng() % int(sqrt(test_count));
    int m = n ? (edges != -1 ? edges : rng() % (n * n)) : 0;
    g = Graph::FromEdgeList(n, {});
    for ([[maybe_unused]] auto _ : views::iota(0, m)) {
      int x = rng() % n;
      int y = rng() % n;
      g.AddEdge(x, y, rng() % 10 + 1);
    }
    return true;
  }
};
}  // namespace

inline auto random_graph(int nodes = -1, int edges = -1) {
  return Catch::Generators::GeneratorWrapper(
      new RandomGraphGenerator(nodes, edges));
}
