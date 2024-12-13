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

TEST_CASE("flow decomposition random graphs", "[fd]") {
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
