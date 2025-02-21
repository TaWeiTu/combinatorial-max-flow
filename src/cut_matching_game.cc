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

class NonStopCutMatchingGame {
 public:
  NonStopCutMatchingGame(const std::vector<CapacityT> &demand_,
                         MatchingPlayer *m);
  std::optional<std::vector<bool>> DoRound();
  std::variant<std::vector<bool>, std::vector<CapacityT>, Expanding> Run();
  std::vector<D> ProjectByF(std::vector<D> r);
  std::vector<D> RandomVectorOrthogonalToOne();

 private:
  const int n_;
  const int rounds_;
  const std::vector<CapacityT> demand_;
  MatchingPlayer *matching_player_;
  const CapacityT total_demand_;

  std::vector<Vertex> A_;  // alive vertices
  std::vector<std::vector<std::tuple<Vertex, Vertex, CapacityT>>> matchings_;

  std::mt19937_64 rng_gen_;
  std::normal_distribution<D> gaussian_;
};

NonStopCutMatchingGame::NonStopCutMatchingGame(
    const std::vector<CapacityT> &demand, MatchingPlayer *m)
    : n_(ssize(demand_)),
      rounds_(10 * pow(log2(n_ + 10), 2)),  // TODO: set rounds more carefully
      demand_(demand),
      matching_player_(m),
      total_demand_(std::accumulate(demand_.begin(), demand_.end(), 0)),
      rng_gen_(std::random_device{}()),
      gaussian_(0, 1) {}

std::vector<D> NonStopCutMatchingGame::RandomVectorOrthogonalToOne() {
  // Project a n-dimensional gaussian distribution to the hyperplane orthogonal
  // to the all ones vector. This gives a gaussian distribution in this
  // hyperplane. Now we could normalize the result to get a unit vector, but
  // this normalization is not necessary as we only look at thresholds in a
  // (scale-invariant) projection later.
  std::vector<D> r(n_);
  for (int i = 0; i < n_; ++i) r[i] = gaussian_(rng_gen_);
  D sum = std::accumulate(r.begin(), r.end(), 0);
  for (int i = 0; i < n_; ++i) r[i] -= sum / n_;
  return r;  // NOTE: not normalized
}

// calculates vector p(v) = ⟨F(v)/demand_[v], r⟩
std::vector<D> NonStopCutMatchingGame::ProjectByF(std::vector<D> r) {
  for (int v = 0; v < n_; ++v) r[v] *= demand_[v];
  for (const auto &M_i : matchings_) {
    auto old_r = r;
    for (const auto &[x, y, c] : M_i) {
      // TODO: this does not quite seem correct to me, double check!
      r[x] += c * (old_r[x] / demand_[x] - old_r[y] / demand_[y]) / 2;
    }
  }
  for (int v = 0; v < n_; ++v) r[v] /= demand_[v];
  return r;
}

std::variant<std::vector<bool>, std::vector<CapacityT>, Expanding>
NonStopCutMatchingGame::Run() {
  for (int i = 0; i < rounds_; ++i) {
    if (auto maybe_balanced_cut = DoRound()) return *maybe_balanced_cut;
  }

  if (std::ssize(A_) == n_) return Expanding{};  // no cuts

  // TODO: is this the right way of calculating the final subdemand_?
  std::vector<CapacityT> subdemand_(n_);
  for (auto v : A_) subdemand_[v] = demand_[v];
  return subdemand_;
}

std::optional<std::vector<bool>> NonStopCutMatchingGame::DoRound() {
  auto r = RandomVectorOrthogonalToOne();
  auto p = ProjectByF(r);
  std::ranges::sort(A_, {}, [&](Vertex v) { return p[v]; });
  // having the source be a prefix and sink suffix OR vice versa leads to
  // potential decrease. Instead of doing something smart to decide which, we
  // simply guess, and are right half of the time. TODO: verify that this works.
  if (rng_gen_() % 2) std::ranges::reverse(A_);

  CapacityT demand_A = 0;
  for (auto v : A_) demand_A += demand_[v];

  auto TakePrefix = [&](CapacityT target, auto it) {
    std::vector<CapacityT> subdemand(n_);
    while (target) {
      subdemand[*it] = std::min(target, demand_[*it]);
      target -= subdemand[*it];
      ++it;
    }
    return subdemand;
  };

  auto source = TakePrefix(demand_A / 8, A_.begin());
  auto sink = TakePrefix((demand_A + 1) / 2, A_.rbegin());

  std::vector<bool> bipartition(n_);
  std::vector<CapacityT> subdemand(n_);

  for (Vertex v = 0; v < n_; ++v) {
    subdemand[v] = abs(source[v] - sink[v]);
    bipartition[v] = (source[v] > sink[v]);
  }

  auto [cut, matching] = matching_player_->Match(subdemand, bipartition);

  CapacityT demand_S = 0;
  for (Vertex v = 0; v < n_; ++v)
    if (cut[v]) demand_S += demand_[v];

  // TODO: check balance factor and constants
  const CapacityT balance_factor = 10 * rounds_;
  if (balance_factor * demand_S >= total_demand_) {
    // found a reasonably balanced cut, return early
    return cut;
  }

  matchings_.emplace_back(matching);

  // remove small cut S from alive vertices A_
  // TODO: check: is this the correct thing to do, or do we only decrease
  // the "alive" demand by source+sink on the S-side?
  std::remove_if(A_.begin(), A_.end(), [&](Vertex v) { return cut[v]; });

  // return no balanced cut, signaling that the cmg should continue
  return {};
}

}  // namespace

std::variant<std::vector<bool>, std::vector<CapacityT>, Expanding>
CutMatchingGame(const std::vector<CapacityT> &demand, MatchingPlayer *m) {
  NonStopCutMatchingGame cmg(demand, m);
  return cmg.Run();
  // TODO: run both forward and backwards cmg and take intersection?
}
