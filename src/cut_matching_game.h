#pragma once

#include <array>
#include <variant>
#include <vector>

#include "types.h"

class MatchingPlayer {
 public:
  // returns [{(cut[0], matching[0]), (cut[1], matching[1])}, scale].
  // the matchings are scaled up by scale
  virtual std::pair<
      std::array<std::pair<std::vector<bool>,
                           std::vector<std::tuple<Vertex, Vertex, CapacityT>>>,
                 2>,
      CapacityT>
  Match(const std::vector<CapacityT> &subdemand,
        const std::vector<bool> &bipartition) = 0;
  MatchingPlayer() = default;
  virtual ~MatchingPlayer() = default;
};

// The interface of a cut-matching game
//
// The Run() function takes a demand d and a matching player and returns one of
// the following:
//   * A balanced sparse cut (std::vector<bool>)
//   * A subdemand d' that is \Omega(1/\log^2 n)-expanding in the union of
//     matchings (std::vector<CapacityT>)
//   * The full demand d is expanding (CutMatchingGame::Expanding)

struct Expanding {};
std::variant<std::vector<bool>, std::vector<CapacityT>, Expanding>
CutMatchingGame(const std::vector<CapacityT> &demand, MatchingPlayer *m);
