#pragma once

#include <initializer_list>
#include <utility>

#include "types.h"

namespace splay {

template <typename U, typename V>
struct SplayNode {
  Vertex id;
  SplayNode *child[2], *fa, *pfa;
  U update;
  V value, aggregation;
  bool reversed;

  template <typename... Args>
  SplayNode(Vertex id, Args &&...args)
      : id(id),
        fa(nullptr),
        pfa(nullptr),
        value(args...),
        aggregation(value),
        reversed(false) {
    child[0] = child[1] = nullptr;
  }

  void Pull() {
    aggregation = value;
    if (child[0]) aggregation = aggregation + child[0]->aggregation;
    if (child[1]) aggregation = child[1]->aggregation + aggregation;
  }

  void Reverse() {
    value = U::Reverse(value);
    aggregation = U::Reverse(aggregation);
    update = U::Reverse(update);
    std::swap(child[0], child[1]);
    reversed = !reversed;
  }

  void Push() {
    for (int d : {0, 1}) {
      if (child[d]) {
        if (reversed) child[d]->Reverse();
        child[d]->Update(update);
      }
    }
    reversed = false;
    update = U();
  }

  bool Relation() const { return this == fa->child[1]; }

  void Rotate() {
    if (fa->fa) fa->fa->Push();
    fa->Push();
    Push();
    std::swap(pfa, fa->pfa);
    bool d = Relation();
    SplayNode *t = fa;
    if (t->fa) t->fa->child[t->Relation()] = this;
    fa = t->fa;
    t->child[d] = child[!d];
    if (child[!d]) child[!d]->fa = t;
    child[!d] = t;
    t->fa = this;
    t->Pull();
    Pull();
  }

  void Splay() {
    while (fa) {
      if (!fa->fa) {
        Rotate();
        continue;
      }
      fa->fa->Push();
      fa->Push();
      if (Relation() == fa->Relation()) {
        fa->Rotate();
        Rotate();
      } else {
        Rotate();
        Rotate();
      }
    }
    Push();
  }

  void Expose() {
    Splay();
    if (child[1]) {
      child[1]->fa = nullptr;
      child[1]->pfa = this;
      child[1] = nullptr;
      Pull();
    }
  }

  bool Splice() {
    Splay();
    if (!pfa) return false;
    pfa->Expose();
    pfa->child[1] = this;
    fa = pfa;
    pfa = nullptr;
    fa->Pull();
    return true;
  }

  void Access() {
    Expose();
    while (Splice());
  }

  void Update(U new_update) {
    value = U::Apply(new_update, value);
    aggregation = U::Apply(new_update, aggregation);
    update = U::Compose(update, new_update);
  }

  void MakeRoot() {
    Access();
    Splay();
    Reverse();
  }
};

}  // namespace splay
