# MCL: the algorithm, and what the measurements showed

Monte Carlo Localization estimates a robot's pose on a known map by
maintaining a weighted particle set. Each cycle:

1. **Motion update** — propagate every particle through the odometry delta
   with noise (`sample_motion_model_odometry`).
2. **Measurement update** — weight every particle by how well the laser scan
   is explained from its pose (`likelihood_field_range_finder_model`).
3. **Resample** — draw a new particle set in proportion to the weights, but
   only when the cloud has lost diversity.

References: Thrun, Burgard & Fox, *Probabilistic Robotics* — Table 5.6
(motion), Table 6.3 (likelihood field), Table 4.4 (low-variance resampling),
ch. 8 (MCL).

**Every number in this document is reproducible** with the evaluation harness
in `tools/evaluate.cpp`, which drives a simulated robot through the shipped map
against known ground truth. The command is given beside each table.

---

## 1. Motion model

The odometry delta is decomposed into rotate–translate–rotate:

```
δ_rot1  = atan2(Δy, Δx) − θ_prev
δ_trans = ‖(Δx, Δy)‖
δ_rot2  = Δθ − δ_rot1
```

each perturbed by zero-mean noise whose standard deviation scales with the
delta magnitudes:

```
σ_rot1  = α₁|δ_rot1| + α₂ δ_trans
σ_trans = α₃ δ_trans + α₄ (|δ_rot1| + |δ_rot2|)
σ_rot2  = α₁|δ_rot2| + α₂ δ_trans
```

Four details are load-bearing, and all four are easy to get wrong:

**Absolute values.** Without them a negative rotation yields a *negative*
standard deviation, which then sign-flips the sample drawn from it; and in the
translation term two opposite rotations cancel instead of accumulating noise.

**Angle wrapping.** `δ_rot1` and `δ_rot2` must be wrapped to [−π, π]. A small
physical rotation that straddles ±π otherwise comes out near 2π, which
multiplies into an enormous σ and detonates the cloud.

**Wrapping the integrated yaw**, so it does not grow without bound over a long
run.

**A near-stationary guard.** When `δ_trans ≈ 0`, `atan2` of pure odometry
noise gives an essentially random `δ_rot1`, injecting spurious rotation while
the robot sits still. Below `motion_min_translation` the rotation split is
skipped and the whole yaw delta is assigned to `δ_rot2`.

---

## 2. Likelihood field

Each beam endpoint is projected into the map and scored by its distance to the
nearest obstacle:

```
p(zₖ) = z_hit · N(dₖ; 0, σ_hit) + z_rand / r_max
```

where `dₖ` is the distance from the endpoint to the nearest occupied cell,
precomputed once per map.

### Units: d², not d⁴

The trap is storing `Δy²_cells + Δx²_cells` — squared distance in **cells** —
and passing it into a Gaussian evaluated as `exp(−x²/2σ²)`. Composed, that
evaluates

$$\exp\left(-\frac{d^4}{2\sigma^2}\right),\ d \text{ in cells}$$

rather than `exp(−d²/2σ²)` with `d` in metres. The field then collapses to
nothing within about one cell, and the filter needs thousands of particles to
localize at all. The fix is to store the true squared distance **in metres**
and have the Gaussian take `d²` directly, so the squaring happens exactly once.

### Computing the field

Comparing every free cell against every occupied cell is O(free × occupied):
tolerable on a toy map, but roughly 10¹⁰ operations on a realistic 2000×2000
one — minutes to hours of startup.

This uses the exact two-pass squared Euclidean distance transform
(Felzenszwalb & Huttenlocher, *Distance Transforms of Sampled Functions*,
Theory of Computing 8(19), 2012): O(cells), and verified bit-exact against
brute force in `test_likelihood_field.cpp`.

One trap worth recording, because the first implementation here hit it: the
lower-envelope step subtracts two sampled values, so seeding non-feature cells
with real `infinity` makes that `inf − inf = NaN`. NaN then fails every
comparison and silently corrupts the whole field. Any row containing no
occupied cell triggers it, which is the common case, not an edge case. Use a
large *finite* sentinel. `LikelihoodField.IsFiniteAndCorrectWithASingleObstacle`
pins this.

### Combining beams: geometric mean, not raw product

The textbook model is a **product** over beams. It tracks acceptably and it is
markedly worse at global localization.

With 30 beams the log-likelihood spread across a uniform cloud is about 120 —
measured over 2000 random free poses on the shipped map, the best pose scores
87 above the cloud mean. `exp` of that difference hands one particle
essentially all the weight and the first resample annihilates the cloud's
diversity. Under the geometric mean the same spread is 4.0, and the best pose
sits 2.9 above the mean.

AMCL addresses the same problem by summing cubes rather than multiplying. This
implementation instead divides the summed log-likelihood by the number of beams
scored — the **geometric mean** per-beam likelihood.

Global localization, 8 seeds each:

```
mcl_evaluate --mode global --particles N --seeds 8          # geometric mean
mcl_evaluate --mode global --particles N --seeds 8 --no-normalize   # product
```

| particles | geometric mean | raw product |
|---|---|---|
| 2 000 | 4/8 converged, 6.49 m | 2/8 converged, 10.77 m |
| 5 000 | **8/8 converged, 0.121 m** | 5/8 converged, 4.16 m |
| 10 000 | 7/8 converged, 2.37 m | 5/8 converged, 4.15 m |

The mean final error is taken over *all* eight seeds, so a single seed that
ends up in the wrong room dominates it; read the converged count first. Note
also that 10 000 particles do worse than 5 000 here: without random particle
injection a bigger cloud is not monotonically better, it just makes a wrong
initial lock heavier to shift.

The geometric mean also has a property the product lacks: the weight no longer
depends on *how many* beams were used, so `beam_skip` is purely a compute knob
instead of silently retuning how peaked the weights are. Tracking error across
beam counts (8 seeds each):

| beams (`--beam-skip`) | geometric mean | raw product |
|---|---|---|
| 90 (4) | 0.054 m | 0.166 m |
| 30 (12) | 0.055 m | 0.139 m |
| 15 (24) | 0.059 m | 0.061 m |
| 7 (48) | 0.058 m | 0.051 m |

The geometric mean holds 0.054–0.059 m across a 13× change in beam count; the
product wanders over 0.051–0.166 m, a range five times wider, and in the same
direction the theory predicts — more beams means a more peaked product, so the
cloud collapses sooner and tracks worse. Set `normalize_by_beam_count: false`
for the textbook product.

### σ_hit

Measured on the shipped map at 0.05 m/cell, 8 seeds:

| σ_hit | tracking error | global @5000 |
|---|---|---|
| 0.05 m | 0.067 m | — |
| **0.10 m** | **0.055 m** | **8/8, 0.121 m** |
| 0.15 m | 0.064 m | 7/8, 1.909 m |
| 0.20 m | 0.089 m | 8/8, 0.126 m |
| 0.35 m | 0.160 m | — |
| 0.50 m | 0.223 m | — |

0.10 m is the default. Note the 0.15 m row: seed-to-seed variance in global
localization is large enough that single runs are not meaningful, which is why
everything here is reported over 8 seeds.

---

## 3. Resampling

Low-variance (systematic) resampling draws a single offset `r ∈ [0, 1/n)` and
takes the particles at cumulative weights `r + m/n`. It is cheaper than `n`
independent draws and introduces less variance.

**Adaptive.** Resampling on *every* update discards diversity for no
information gain and converges prematurely. This resamples only when the
effective sample size

$$N_\text{eff} = \frac{1}{\sum_i w_i^2}$$

drops below `resample_threshold_ratio × n`.

**Weights are reset to 1/n afterwards.** After resampling the cloud's
*density* carries the distribution, so the weights must not also carry it.

**The index bound.** The natural way to write the inner loop is:

```cpp
while (u > c && m < n) { i++; c += source[i].weight; }
```

which guards on `m`, the *outer* loop variable — and `m` never changes inside
the inner loop, so the condition is constant and `i` can walk past the last
particle. Since normalized weights sum to 1.0 only up to floating-point error,
`u > c` at the final particle is a live possibility, not a theoretical one. The
guard belongs on `i`. The resampler is a free function here precisely so this
can be tested directly, including the all-weight-on-the-last-particle case.

One honest edge: with `r` exactly 0 the comparison `0 > 0` is false, so the
first particle is emitted even if its weight is zero. That is the textbook
algorithm's behaviour, it costs at most one sample out of n, and the test
asserts what the algorithm actually guarantees rather than papering over it.

---

## 4. Pose estimate

The reported pose is the **weighted** mean, with a **circular** mean for yaw:

$$\bar\theta = \operatorname{atan2}\left(\sum_i w_i \sin\theta_i,\ \sum_i w_i \cos\theta_i\right)$$

Averaging yaw arithmetically is wrong near the branch cut. On a cloud seeded at
π with 0.3 rad spread, the circular mean is 3.131 rad and the arithmetic mean
is 0.199 rad — nearly a half-turn off, pointing the wrong way entirely.

A covariance is published alongside the mean. This matters because the mean of
a multi-modal cloud sits in no mode at all; without the covariance a confident
wrong answer is indistinguishable from a correct one.

---

## 5. Measured performance

Shipped map (12 × 10 m at 0.05 m/cell, three rooms plus an alcove), a 20.25 m
route, 30 of 360 beams, odometry drifting 0.31–0.95 m end to end across the
eight seeds. 8 seeds.

```
mcl_evaluate --map map/map.yaml --mode track  --particles N --seeds 8
mcl_evaluate --map map/map.yaml --mode global --particles N --seeds 8
```

**Tracking from a seeded prior — reliable at every count tested.**

| particles | converged | mean final error |
|---|---|---|
| 200 | 8/8 | 0.063 m |
| 500 | 8/8 | 0.062 m |
| 1 000 | 8/8 | 0.055 m |

**Global localization from a uniform cloud — needs ~5 000 particles.**

| particles | converged | mean final error |
|---|---|---|
| 500 | 1/8 | 11.30 m |
| 1 000 | 2/8 | 5.83 m |
| 2 000 | 4/8 | 6.49 m |
| 5 000 | **8/8** | 0.121 m |
| 10 000 | 7/8 | 2.37 m |

The error column averages over all eight seeds, converged or not, so it tracks
the converged count rather than the accuracy of a successful run: at 5 000,
where every seed converges, it is 0.121 m.

Global localization is a genuinely harder problem than tracking: the filter has
to resolve which room it is in before it can refine a pose, and with too few
particles no sample lands close enough to the truth for the measurement model
to reward it. The map's asymmetry — three differently proportioned rooms, an
alcove, two off-centre pillars — is what makes 5 000 particles enough. A
symmetric floorplan leaves mirrored poses that explain the scan equally well,
and the filter can lock onto the wrong one and never recover.

### What is not implemented

- **KLD-adaptive sample size** (Fox, *Adapting the Sample Size in Particle
  Filters Through KLD-Sampling*, 2003) — many particles while the posterior is
  spread out, few once it is concentrated. This would get global localization's
  5 000 particles down to tracking's few hundred automatically.
- **Random particle injection** keyed on the short- and long-term average
  weight (AMCL's `recovery_alpha_slow` / `recovery_alpha_fast`), which lets the
  filter recover from a wrong lock instead of staying in it.
- **Cluster-based reporting** — publish the largest cluster's mean rather than
  the mean over all particles, as AMCL does. Until this exists, read the
  published covariance: it is what distinguishes a converged estimate from a
  mean sitting between two modes.
