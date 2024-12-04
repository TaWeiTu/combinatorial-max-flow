#pragma once

#include <vector>

using Vertex = int;
using Edge = int;
using CapacityT = int64_t;

struct Graph {
  std::vector<std::vector<Edge>> out_edges;
  std::vector<std::vector<Edge>> in_edges;
  std::vector<CapacityT> cap;
  std::vector<Vertex> head, tail;

  Graph Residual(const std::vector<CapacityT> &flow) const;

  void AddEdge(Vertex h, Vertex t) {
    out_edges[h].push_back(head.size());
    in_edges[t].push_back(head.size());
    head.push_back(h);
    tail.push_back(t);
    cap.push_back(0);
  }
};
