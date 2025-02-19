#include "cut_matching_game.h"

#include <algorithm>
#include <cassert>
#include <optional>
#include <random>
#include <tuple>
#include <variant>
#include <vector>

namespace {

using D = long double;

struct NonStopCutMatchingGame {
  const int n;
  const int rounds;
  const std::vector<CapacityT> demand;
  MatchingPlayer *matching_player;
  const CapacityT total_demand;

  std::vector<Vertex> A;  // alive vertices
  std::vector<std::vector<std::tuple<Vertex, Vertex, CapacityT>>> matchings;

  std::mt19937_64 rng_gen;
  std::uniform_real_distribution<D> uniform01;

  NonStopCutMatchingGame(const std::vector<CapacityT> &demand,
                         MatchingPlayer *m);
  std::optional<std::vector<bool>> DoRound();
  std::variant<std::vector<bool>, std::vector<CapacityT>, Expanding> Run();
  std::vector<D> project_F(std::vector<D> r);
};

NonStopCutMatchingGame::NonStopCutMatchingGame(
    const std::vector<CapacityT> &demand, MatchingPlayer *m)
    : n(ssize(demand)),
      rounds(10 * log2(n) * log2(n) + 10),  // TODO: set #rounds more carefully
      demand(demand),
      matching_player(m),
      total_demand(std::accumulate(demand.begin(), demand.end(), 0)),
      rng_gen(42),
      uniform01(0, 1) {}

// calculates vector p(v) = ⟨F(v)/demand[v], r⟩
std::vector<D> NonStopCutMatchingGame::project_F(std::vector<D> r) {
  for (int v = 0; v < n; ++v) r[v] *= demand[v];
  for (const auto &M_i : matchings) {
    auto old_r = r;
    for (const auto &[x, y, c] : M_i) {
      r[x] += c * (old_r[x] / demand[x] - old_r[y] / demand[y]) / 2;
    }
  }
  for (int v = 0; v < n; ++v) r[v] /= demand[v];
  return r;
}

std::variant<std::vector<bool>, std::vector<CapacityT>, Expanding>
NonStopCutMatchingGame::Run() {
  for (int i = 0; i < rounds; ++i) {
    if (auto maybe_balanced_cut = DoRound()) return *maybe_balanced_cut;
  }
  // TODO: construct witness graph?

  if (std::ssize(A) == n) return Expanding{};  // no cuts

  // TODO: is this the right way of calculating the final subdemand?
  std::vector<CapacityT> subdemand(n);
  for (auto v : A) subdemand[v] = demand[v];
  return subdemand;
}

std::optional<std::vector<bool>> NonStopCutMatchingGame::DoRound() {
  // TODO: do we need <r,1> = 0, or is independent U(0,1) enough?
  // I think U(0,1) is enough, as it is only used later to find a threshold
  // value after projection.
  std::vector<D> r(n);
  for (int v = 0; v < n; ++v) r[v] = uniform01(rng_gen);
  auto p = project_F(r);

  std::ranges::sort(A, {}, [&](Vertex v) { return p[v]; });
  // having the source be a prefix and sink suffix OR vice versa leads to
  // potential decrease. Instead of doing something smart to decide which, we
  // simply guess, and are right half of the time. TODO: verify that this works.
  if (rng_gen() % 2) std::ranges::reverse(A);

  CapacityT demand_of_A = 0;
  for (auto v : A) demand_of_A += demand[v];

  auto TakePrefix = [&](CapacityT target, auto it) {
    std::vector<CapacityT> subdemand(n);
    while (target) {
      subdemand[*it] = std::min(target, demand[*it]);
      target -= subdemand[*it];
      ++it;
    }
    return subdemand;
  };

  auto source = TakePrefix(demand_of_A / 8, A.begin());
  auto sink = TakePrefix((demand_of_A + 1) / 2, A.rbegin());

  std::vector<bool> bipartition(n);
  std::vector<CapacityT> subdemand(n);

  for (Vertex v = 0; v < n; ++v) {
    subdemand[v] = abs(source[v] - sink[v]);
    bipartition[v] = (source[v] > sink[v]);
  }

  auto [cut, matching] = matching_player->Match(subdemand, bipartition);

  CapacityT demand_S = 0;
  for (Vertex v = 0; v < n; ++v)
    if (cut[v]) demand_S += demand[v];

  // TODO: check balance factor and constants
  const CapacityT balance_factor = 10 * rounds;
  if (balance_factor * demand_S >= total_demand) {
    // found a reasonably balanced cut, return early
    return cut;
  }

  matchings.emplace_back(matching);

  // remove small cut S from alive vertices A
  // TODO: check: is this the correct thing to do, or do we only decrease
  // the "alive" demand by source+sink on the S-side?
  std::remove_if(A.begin(), A.end(), [&](Vertex v) { return cut[v]; });

  // return no balanced cut, signaling that the cmg should continue
  return {};
}

}  // namespace

std::variant<std::vector<bool>, std::vector<CapacityT>, Expanding>
CutMatchingGame(int n, const std::vector<CapacityT> &demand,
                MatchingPlayer *m) {
  assert(n == ssize(demand));  // TODO: remove n as parameter?
  NonStopCutMatchingGame cmg(demand, m);
  return cmg.Run();
  // TODO: run both forward and backwards cmg and take intersection?
}
