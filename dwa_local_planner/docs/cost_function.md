# The DWA cost function

## What DWA scores

At each control cycle the planner samples the *dynamic window* — the set of
velocity pairs `(v, ω)` reachable within one timestep `dt` under the
acceleration limits:

```
v   ∈ [max(v₀ − a_max·dt, v_min),  min(v₀ + a_max·dt, v_max)]
ω   ∈ [max(ω₀ − α_max·dt, −ω_max), min(ω₀ + α_max·dt,  ω_max)]
```

Each sampled pair is forward-simulated as a *constant* control over the
prediction horizon `T` using the unicycle model:

```
θ_{k+1} = θ_k + ω·dt
x_{k+1} = x_k + v·cos(θ_{k+1})·dt
y_{k+1} = y_k + v·sin(θ_{k+1})·dt
```

The resulting trajectory is scored on four terms, and the lowest-cost
collision-free candidate wins. Its `(v, ω)` is issued for one `dt`, then the
whole thing repeats.

Reference: Fox, Burgard & Thrun, "The Dynamic Window Approach to Collision
Avoidance", *IEEE Robotics & Automation Magazine* 4(1), 1997.

## The four terms

All four are defined to be non-negative, and lower is better.

| Term | Definition | Units |
|---|---|---|
| `goalDistance` | `‖p_end − goal‖` | m |
| `heading` | `\|wrapToPi(atan2(goal−p_end) − θ_end)\|` | rad |
| `clearance` | `1 / min_k min_j ‖p_k − o_j‖`, or `+∞` on collision | 1/m |
| `velocity` | `max(0, v_max − v_end)` | m/s |

Two details matter.

**The heading term is measured at the robot.** It is the bearing error between
where the robot is pointing and the direction to the goal, both evaluated at
the trajectory endpoint. An earlier version instead took the angle between the
`origin → goal` and `origin → endpoint` vectors, which makes the cost depend
on where the world origin happens to sit: minimizing it drives the endpoint
onto the ray from the origin through the goal rather than toward the goal. It
only looked correct in the shipped demo because that start/goal pair happened
to straddle the origin. The `offset` scenario is the regression test.

**Clearance checks every pose.** Subsampling the trajectory (an earlier version
stepped by 2) lets a thin obstacle fall between checked poses, so the robot
can tunnel through it. The `corridor` scenario covers this.

## Normalization, and why sum-normalization fails here

The four terms have incomparable units (m, rad, 1/m, m/s) and unbounded
magnitudes, so they cannot be summed raw with any principled set of weights.
Each is therefore normalized across the candidate set before weighting.

The implementation uses **min-max** normalization:

```
ĉᵢ = (cᵢ − min c) / (max c − min c)        (0 if the range is degenerate)
```

so every term contributes `weight × [0, 1]` and the weights express relative
priority directly.

The other obvious choice — dividing each term by its sum over the candidate
set — **destroys the weights' meaning at this scale**, and it is worth
recording why, because it is not obvious.

Sum-normalization preserves each term's *relative* spread. With the default
limits the dynamic window is extremely narrow: `a_max·dt = 0.2 × 0.1 = 0.02`
m/s. Over a 4 s horizon the candidates therefore differ in position by at most
a few centimetres, while they differ in final *heading* by up to
`2·α_max·dt·T ≈ 0.56` rad. Measured on the `default` scenario at startup, over
the 231 collision-free candidates:

| Term | Raw mean | Raw range | Range ÷ sum |
|---|---|---|---|
| heading | 0.39 rad | 0.56 rad | **6.2 × 10⁻³** |
| velocity | 1.0 m/s | 0.040 m/s | 1.7 × 10⁻⁴ |
| clearance | 0.17 /m | 0.0024 /m | 6.0 × 10⁻⁵ |
| goalDistance | 32.5 m | 0.15 m | 2.1 × 10⁻⁵ |

The goal-distance term has a large mean and a tiny range, so dividing by its
sum dilutes it into irrelevance; the heading term has a small mean and a large
relative range, so it survives and outweighs goal distance by a factor of
roughly 300. Whatever the four weights are set to, the heading term decides,
which is precisely what a normalization step is supposed to prevent.

What that does *not* do is stall the robot, and it is worth being exact about
why. The heading cost at the endpoint is `|bearing − (θ₀ + ω·T)|`: it depends
on `ω` alone, not on `v`. So all 11 velocity samples sharing the best `ω` tie
on the dominant term, and the diluted terms break the tie in favour of the
fastest one. Run end to end with sum-normalization and weights all 1.0, the
planner still reaches the goal on `default` in 344 cycles at full speed —
indistinguishable from the shipped planner's 345, and for a reason that has
nothing to do with the weights being right.

The stall is real, but it comes from *actually* over-weighting heading or
clearance, which min-max normalization makes possible because every term then
spans the full `[0, 1]`:

| Weighting | `default` |
|---|---|
| heading only (0, 1, 0, 0) | never reaches the goal in 2000 cycles; peak forward speed 0.14 m/s against a 1.0 m/s limit, and it reverses away from the start |
| 1, 2, 1, 0.5 | stalls at the start: `v = 0`, 0.00 m travelled in 2000 cycles |
| 1, 1, 2, 1 | stalls at the start, likewise |

Min-max normalization removes the dependence on each term's mean magnitude,
which is what makes the weights portable across scenarios — and what makes
mis-weighting show up as visibly broken behaviour instead of a term silently
dropping out.

## Weights

Defaults are all `1.0`. This is not laziness — the balance matters:

| Weights (goal, heading, clear, vel) | `default` | `offset` |
|---|---|---|
| 1, 1, 1, 1 | reaches goal, 345 cycles | reaches goal, 257 |
| 2, 1, 1, 1 | reaches goal, 345 | reaches goal, 260 |
| 1, 2, 1, 0.5 | **stalls at start** | reaches goal, 283 |
| 1, 1, 2, 1 | **stalls at start** | reaches goal, 266 |

Over-weighting heading or clearance relative to velocity reproduces the stall:
because a single cycle can change heading much more than position, a planner
that prizes either one over making progress will sit still and point at the
goal forever. If you retune these, check both navigation scenarios — `offset`
alone will not catch it.

## Collision handling and recovery

`clearanceCost` returns `+∞` for a colliding trajectory, and those candidates
are dropped *before* normalization — otherwise the normalizer itself becomes
infinite.

If every candidate collides, `plan()` returns `std::nullopt`. This is reachable:
the robot only needs to already be inside an obstacle's collision radius, after
which even pure in-place rotation collides. Callers must handle it;
`recoveryControl()` decelerates toward zero and rotates in place, both clamped
to the admissible window. An earlier version indexed element `[1]` of the empty
result instead, which is undefined behaviour. The `trapped` scenario covers it.
