#include "../src/weighted_push_relabel.h"

#include <catch2/catch_all.hpp>
#include <random>

#include "correct_flow.h"
#include "random_graph_generator.h"

using namespace std;

TEST_CASE("weighted push relabel small fixed graph", "[wpr]") {
  auto g = Graph::FromEdgeList(3, {{1, 2, 3}, {1, 0, 9}, {0, 2, 4}});

  auto [flow_value, flow, residual_demand] =
      WeightedPushRelabel(g, {0, 100, -100}, {1, 3, 4}, 100);

  CAPTURE(flow_value, flow, residual_demand);
  REQUIRE(flow_value == 7);
  REQUIRE(flow == vector<CapacityT>{3, 4, 4});
  REQUIRE(residual_demand == vector<CapacityT>{0, 93, -93});
}

TEST_CASE("weighted push relabel stress", "[wpr][stress]") {
  mt19937_64 rng(Catch::rngSeed());
  auto random_vector = [&](int n, auto lo, auto hi) {  // in [lo,hi)
    vector<decltype(lo)> v(n);
    for (auto& a : v) a = rng() % (hi - lo) + lo;
    return v;
  };

  // run on 200 graphs where n <= 15, and 3 where n=50
  auto g = GENERATE(take(200, random_graph()), take(3, random_graph(50)));
  auto demand = random_vector(g.n, CapacityT(-10), CapacityT(10));
  auto weights = random_vector(g.m, WeightT(1), WeightT(10));

  CAPTURE(g, demand, weights);

  SECTION("is correct when large height") {
    WeightT h =
        min({g.n * 10, accumulate(begin(weights), end(weights), 0)}) / 3 + 1;

    auto [flow_value, flow, residual_demand] =
        WeightedPushRelabel(g, demand, weights, h);

    REQUIRE(flow_value == CorrectFlowImplementation::MaximumFlow(g, demand));

    CapacityT demand_routed = 0;
    for (auto v : g.Vertices()) {
      demand_routed = max<CapacityT>(0, demand[v] - residual_demand[v]);
      // check subdemand
      REQUIRE(abs(residual_demand[v]) <= abs(demand[v]));
      REQUIRE(residual_demand[v] * demand[v] >= 0);
      // check flow and residual demand agree
      for (auto e : g.out_edges[v]) demand[v] -= flow[e];
      for (auto e : g.in_edges[v]) demand[v] += flow[e];
      REQUIRE(demand[v] == residual_demand[v]);
    }
    REQUIRE(demand_routed == flow_value);
  }
}
