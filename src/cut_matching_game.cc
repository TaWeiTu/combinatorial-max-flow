#include "cut_matching_game.h"

#include <algorithm>
#include <cassert>
#include <deque>
#include <iostream>
#include <optional>
#include <random>
#include <ranges>
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
  std::deque<std::vector<std::tuple<Vertex, Vertex, D>>> matchings_;

  std::mt19937_64 rng_gen_;
  std::normal_distribution<D> gaussian_;
};

NonStopCutMatchingGame::NonStopCutMatchingGame(
    const std::vector<CapacityT> &demand, MatchingPlayer *m)
    : n_(std::ssize(demand)),
      rounds_(2 * pow(log2(n_ + 10), 2)),  // TODO: set rounds more carefully
      demand_(demand),
      matching_player_(m),
      total_demand_(std::accumulate(demand.begin(), demand.end(), 0)),
      A_(n_),
      rng_gen_(42),  // rng_gen_(std::random_device{}()), // TODO: change back?
      gaussian_(0, 1) {
  std::iota(A_.begin(), A_.end(), 0);
}

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
      // r[x] += c * (old_r[x] / demand_[x] - old_r[y] / demand_[y]) / 2;
      // auto flow = c * (old_r[x] / demand_[x] - old_r[y] / demand_[y]) / 2;
      auto flow = c / D(2 * demand_[x]) * old_r[x];
      r[x] -= flow;
      r[y] += flow;
    }
  }
  for (int v = 0; v < n_; ++v) {
    if (demand_[v] == 0)
      r[v] = 0;
    else
      r[v] /= demand_[v];
  }
  return r;
}

std::variant<std::vector<bool>, std::vector<CapacityT>, Expanding>
NonStopCutMatchingGame::Run() {
  for (int i = 0; i < rounds_; ++i) {
    if (auto maybe_balanced_cut = DoRound()) return *maybe_balanced_cut;
  }

  if (std::ssize(A_) == n_) return Expanding{};  // no cuts

  // TODO: is this the right way of calculating the final subdemand?
  std::vector<CapacityT> subdemand(n_);
  for (auto v : A_) subdemand[v] = demand_[v];
  return subdemand;
}

std::optional<std::vector<bool>> NonStopCutMatchingGame::DoRound() {
  auto r = RandomVectorOrthogonalToOne();
  auto p = ProjectByF(r);
  std::ranges::sort(A_, {}, [&](Vertex v) { return p[v]; });

  // having the source be a prefix and sink suffix OR vice versa leads to
  // potential decrease. If we simply assume the prefix is the source, we are
  // right half of the time. TODO: verify that this works.

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

  if (false) {  // TODO: remove debug info
    std::cerr << "F=\n";
    for (auto i : std::views::iota(0, n_)) {
      std::vector<D> ei(n_);
      ei[i] = 1;
      ei = ProjectByF(ei);
      for (auto a : ei) std::cerr << a << " ";
      std::cerr << std::endl;
    }
    std::cerr << "d[";
    for (auto a : demand_) std::cerr << a << " ";
    std::cerr << "]\n";
    std::cerr << "s[";
    for (auto a : source) std::cerr << a << " ";
    std::cerr << "]\n";
    std::cerr << "t[";
    for (auto a : sink) std::cerr << a << " ";
    std::cerr << "]\n";
  }

  std::vector<bool> bipartition(n_);
  std::vector<CapacityT> subdemand(n_);

  for (Vertex v = 0; v < n_; ++v) {
    subdemand[v] = abs(source[v] - sink[v]);
    bipartition[v] = (source[v] <= sink[v]);
  }

  auto [cut_and_matching, scale] =
      matching_player_->Match(subdemand, bipartition);

  // Check if one of the cuts is balanced.
  for (int rev = 0; rev < 2; ++rev) {
    CapacityT demand_S = 0;
    for (Vertex v = 0; v < n_; ++v)
      if (cut_and_matching[rev].first[v]) demand_S += demand_[v];

    // TODO: check balance factor and constants
    const CapacityT balance_factor = 10 * rounds_;
    if (balance_factor * demand_S >= total_demand_) {
      // found a reasonably balanced cut, return early
      assert(count(cut_and_matching[rev].first.begin(),
                   cut_and_matching[rev].first.end(), false) &&
             count(cut_and_matching[rev].first.begin(),
                   cut_and_matching[rev].first.end(), true) &&
             "must be non-trivial cut");
      return cut_and_matching[rev].first;
    }
  }

  // To certify both out and in expansion, we multiply the matchings (both
  // directions) both to the front and back of the flow matrix F.
  // TODO: verify if this is correct.
  //  (at least it makes the flow matrix look uniform at the end)
  std::vector<std::tuple<Vertex, Vertex, D>> matching;
  for (int rev = 0; rev < 2; ++rev) {
    for (const auto &[x, y, c] : cut_and_matching[rev].second) {
      matching.emplace_back(x, y, D(c) / D(scale));
    }
  }
  matchings_.emplace_back(matching);
  matchings_.emplace_front(matching);

  // remove small cut S from alive vertices A_
  // TODO: check: is this the correct thing to do, or do we only decrease
  // the "alive" demand by source+sink on the S-side?
  std::ignore = std::remove_if(A_.begin(), A_.end(), [&](Vertex v) {
    return cut_and_matching[0].first[v] || cut_and_matching[1].first[v];
  });

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
