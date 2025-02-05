#include "../src/graph.h"

#include <catch2/catch_all.hpp>
#include <queue>
#include <random>

#include "random_graph_generator.h"

TEST_CASE("strongly connected components", "[scc][stress]") {
  std::mt19937_64 rng(Catch::rngSeed());

  auto g = GENERATE(take(200, random_graph()), take(100, random_graph(50)),
                    take(30, random_graph(200)));

  CAPTURE(g);

  SECTION("is correct") {
    auto scc = g.SCC();

    auto ReachableSet = [&](Vertex x) {
      std::vector<bool> reachable(g.n);
      std::queue<Vertex> que;
      que.push(x);
      reachable[x] = true;
      while (!que.empty()) {
        Vertex u = que.front();
        que.pop();
        for (Edge e : g.out_edges[u]) {
          if (!reachable[g.head[e]]) {
            reachable[g.head[e]] = true;
            que.push(g.head[e]);
          }
        }
      }
      return reachable;
    };

    std::vector<std::vector<bool>> closure(g.n);
    for (Vertex u : g.Vertices()) {
      closure[u] = ReachableSet(u);
    }
    for (Vertex x : g.Vertices()) {
      for (Vertex y : g.Vertices()) {
        REQUIRE((closure[x][y] && closure[y][x]) == (scc[x] == scc[y]));
      }
    }
    for (Edge e : g.Edges()) {
      REQUIRE(scc[g.tail[e]] <= scc[g.head[e]]);
    }
  }
}
