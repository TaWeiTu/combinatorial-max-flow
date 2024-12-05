#pragma once

#include "types.h"

namespace splay {

template <typename U, typename V>
struct SplayNode {
  Vertex id;
  SplayNode *child[2], *fa, *pfa;
  U update;
  V value, aggregation;

  template <typename... Args>
  SplayNode(Vertex id, Args &&...args)
      : id(id),
        fa(nullptr),
        pfa(nullptr),
        value(args...),
        aggregation(value) {
    child[0] = child[1] = nullptr;
  }

  void Pull() {
    aggregation = value;
    for (int d : {0, 1}) {
      if (child[d]) {
        aggregation = aggregation + child[d]->aggregation;
      }
    }
  }

  void Push() {
    for (int d : {0, 1}) {
      if (child[d]) {
        child[d]->value = U::Apply(update, child[d]->value);
        child[d]->aggregation = U::Apply(update, child[d]->aggregation);
        child[d]->update = U::Compose(child[d]->update, update);
      }
    }
    update = U();
  }

  bool Relation() const { return this == fa->child[1]; }

  void Rotate() {
    if (fa->fa) {
      fa->fa->Push();
    }
    fa->Push();
    Push();
    std::swap(pfa, fa->pfa);
    bool d = Relation();
    SplayNode *t = fa;
    if (t->fa) {
      t->fa->child[t->Relation()] = this;
    }
    fa = t->fa;
    t->child[d] = child[!d];
    if (child[!d]) {
      child[!d]->fa = t;
    }
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
  }

  void Evert() {
    Access();
    Splay();
  }

  void Expose() {
    Splay();
    Push();
    if (child[1]) {
      child[1]->fa = nullptr;
      child[1]->pfa = this;
      child[1] = nullptr;
      Pull();
    }
  }

  bool Splice() {
    Splay();
    if (!pfa) {
      return false;
    }
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

  V Query() const { return aggregation; }

  void Update(U new_update) {
    value = U::Apply(new_update, value);
    aggregation = U::Apply(new_update, aggregation);
    update = U::Compose(update, new_update);
  }
};

}  // namespace splay
