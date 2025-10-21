
# Combinatorial Maximum Flow
This repository contains an implementation of maximum flow on directed graphs with a theoretical time complexity of $\tilde{O}(n^2)$.

> [!NOTE]
> This implementation is primarily academic and not optimized for practical performance on larger maximum flow instances. The $\tilde{O}$-notation conceals a substantial $\log^{19}(n)$ factor, resulting in an actual theoretical running time of $O(n^2 \log^{19}(n))$.
>
> The repository serves as a proof of concept demonstrating that the $\tilde{O}(n^2)$ algorithm can be implemented. To our knowledge, few (if any) maximum flow algorithms with theoretical running times faster than Goldberg-Rao's (1997) classic $\tilde{O}(\min(\sqrt{m},n^{2/3})m)$ algorithm have been fully implemented, due to their reliance on complex data structures and continuous optimization techniques.

The implementation is based on the following papers:
* [[BBST24]](#BBST24): Maximum Flow by Augmenting Paths in $n^{2+o(1)}$ Time.
* [[BBLST25]](#BBLST25): Combinatorial Maximum Flow via Weighted Push-Relabel on Shortcut Graphs.

> [!NOTE]
> The current repository contains a slightly older and different version of the algorithm compared to the one described in [[BBLST25]](#BBLST25). We will update the implementation shortly.

## Details
For complete details, please refer to [[BBLST25]](#BBLST25) (which builds upon [[BBST24]](#BBST24)). A brief overview of the algorithm follows:

> [!NOTE]
> While our implementation is complete, it uses parameter settings that differ from those specified in the papers. These modifications appear to improve performance (at least on small graphs), as implementing the exact parameters from the paper would be impractical due to the $\log^{25}(n)$ factor.
>
> The current implementation successfully computes maximum flows on graphs where $n \approx 30$ in a few seconds.

The main algorithm finds a constant approximation of the flow, then iterates $O(\log n)$ times on the residual graph until it converges to an optimal exact maximum flow.

### Weighted Push Relabel that implements [BBST24, Algorithm 1]
Given additional hints: (i) edge lengths $w : E \mapsto \mathbb{Z}$ and (ii) a height parameter $h$ such that there exists a maximum flow using only paths of $w$-length at most $h$, this algorithm can find an $O(1)$-approximate maximum flow in $O(\sum_{e \in E} \frac{h \log n}{w(e)})$ time.

If we can construct edge lengths $w$ such that:
* $\sum_{e} \frac{1}{w(e)} = \tilde{O}(n)$, and
* there exists a maximum flow of $w$-length at most $\tilde{O}(n)$,

then we achieve the desired $\tilde{O}(n^2)$ running time.

The weighted push relabel algorithm itself is relatively simple and efficient in practice. The challenge lies in constructing edge lengths with the desired properties.

### Expander Hierarchy
Both papers construct these hints (edge lengths $w$) by building an expander hierarchy on the graph. The construction in [[BBST24]](#BBST24) involves a complex approach, while [[BBLST25]](#BBLST25) significantly simplifies this by introducing shortcuts. Our implementation follows the latter approach.

The final algorithm finds a flow not on the original graph $G$, but on $G \cup A$ where $A$ is a set of "expanding" shortcuts. As a post-processing step, the algorithm "unfolds" this flow back to $G$ with only an $O(1)$ loss in the approximation factor.

## Building
Initialize cmake:
```
cmake -S . -B build
```

Build:
```
cmake --build build -j 8
```

Build and Run Tests:
```
cmake --build build -j 8 --target test
```

## References
* **[BBST24]** <br> <a name="BBST24"></a> Maximum Flow by Augmenting Paths in $n^{2+o(1)}$ Time. <br>
Aaron Bernstein, Joakim Blikstad, Thatchaphol Saranurak, Ta-Wei Tu. <br>
FOCS 2024. <br>
[[arXiv]](https://arxiv.org/abs/2406.03648)
[[proceedings]](https://doi.org/10.1109/FOCS61266.2024.00123)
[[slides for a high-level overview]](https://blikstad.gitlab.io/slides/Combinatorial_Maximum_Flow.pdf)
[[video 30min]](https://youtu.be/K3RgpJmgmUI)

* **[BBLST25]** <br> <a name="BBLST25"></a> Combinatorial Maximum Flow via Weighted Push-Relabel on Shortcut Graphs. <br>
Aaron Bernstein, Joakim Blikstad, Jason Li, Thatchaphol Saranurak, Ta-Wei Tu. <br>
FOCS 2025. <br>
[[arXiv]](https://arxiv.org/abs/2510.17182)
