#include "../src/flow_util.h"

#include <catch2/catch_all.hpp>
#include <random>

#include "random_graph_generator.h"

namespace {

std::vector<CapacityT> DemandRoutedByFlow(const Graph &g,
                                          const std::vector<CapacityT> &flow) {
  std::vector<CapacityT> demand(g.n);
  for (Edge e : g.Edges()) {
    demand[g.tail[e]] += flow[e];
    demand[g.head[e]] -= flow[e];
  }
  return demand;
}

std::vector<CapacityT> ToSingleCommodity(
    const Graph &g, const std::map<std::pair<Vertex, Vertex>, CapacityT> &md) {
  std::vector<CapacityT> demand(g.n);
  for (auto [k, d] : md) {
    demand[k.first] += d;
    demand[k.second] -= d;
  }
  return demand;
}

}  // namespace

TEST_CASE("flow decomposition random graphs", "[fd][stress]") {
  std::mt19937_64 rng(Catch::rngSeed());
  auto random_vector = [&](int n, auto lo, auto hi) {  // in [lo,hi)
    vector<decltype(lo)> v(n);
    for (auto &a : v) a = rng() % (hi - lo) + lo;
    return v;
  };

  auto g = GENERATE(take(200, random_graph()), take(100, random_graph(50)),
                    take(30, random_graph(200)));
  auto flow = random_vector(g.m, CapacityT(0), CapacityT(10));

  CAPTURE(g, flow);

  SECTION("is correct") {
    FlowDecomposition fd(g, flow);
    auto demand = fd.Demand();
    decltype(demand) subdemand;
    for (auto [k, v] : demand) {
      subdemand[k] = rng() % (v + 1);
    }
    auto subflow = fd.Route(subdemand);
    REQUIRE(DemandRoutedByFlow(g, subflow) == ToSingleCommodity(g, subdemand));
    REQUIRE(std::ranges::all_of(std::ranges::views::iota(0, g.m),
                                [&](int i) { return subflow[i] <= flow[i]; }));
  }
}

TEST_CASE("flow rounding", "[fr]") {
  SECTION("is correct") {
    Graph g(4);
    g.AddEdge(0, 1, 0);
    g.AddEdge(1, 3, 0);
    g.AddEdge(0, 2, 0);
    g.AddEdge(2, 3, 0);
    auto f = FlowRoundingExact(g, {1, 1, 1, 1}, 2);
  }
}

TEST_CASE("flow rounding random graphs", "[fr][stress]") {
  std::mt19937_64 rng(Catch::rngSeed());
  auto random_vector = [&](int n, auto lo, auto hi) {  // in [lo,hi)
    vector<decltype(lo)> v(n);
    for (auto &a : v) a = rng() % (hi - lo) + lo;
    return v;
  };

  auto n = GENERATE(take(200, random(5, 20)), take(20, random(50, 200)));
  Graph g(n);
  auto demand = random_vector(g.n - 1, 0, 3);
  demand.push_back(-std::accumulate(demand.begin(), demand.end(), 0));
  CapacityT scale = rng() % 3 + 2;
  for (auto &v : demand) v *= scale;
  std::vector<CapacityT> flow;
  while (true) {
    std::vector<Vertex> source, sink;
    for (Vertex v : g.Vertices()) {
      if (demand[v] > 0)
        source.push_back(v);
      else if (demand[v] < 0)
        sink.push_back(v);
    }
    if (source.empty() || sink.empty()) {
      assert(source.empty() && sink.empty());
      break;
    }
    Vertex s = source[rng() % source.size()], t = sink[rng() % sink.size()];
    int k = rng() % g.n;
    CapacityT f = rng() % std::min(demand[s], -demand[t]) + 1;
    demand[s] -= f;
    demand[t] += f;
    for (int i = 0; i < k; ++i) {
      Vertex v = rng() % g.n;
      g.AddEdge(s, v, 0);
      flow.push_back(f);
      s = v;
    }
    g.AddEdge(s, t, 0);
    flow.push_back(f);
  }

  CAPTURE(g, flow, scale);

  SECTION("is correct") {
    auto rounded_flow = FlowRoundingExact(g, flow, scale);
    REQUIRE(std::ranges::all_of(g.Edges(), [&](Edge e) {
      return rounded_flow[e] <= (flow[e] + scale - 1) / scale * scale;
    }));
  }
}
