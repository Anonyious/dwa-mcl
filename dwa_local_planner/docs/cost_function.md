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
set — **does not work at this scale**, and it is worth recording why, because
it is not obvious and it produces a planner that looks plausible and then
quietly refuses to move.

Sum-normalization preserves each term's *relative* spread. With the default
limits the dynamic window is extremely narrow: `a_max·dt = 0.2 × 0.1 = 0.02`
m/s. Over a 4 s horizon the candidates therefore differ in position by at most
a few centimetres, while they differ in final *heading* by up to
`2·α_max·dt·T ≈ 0.56` rad. Measured on the `default` scenario at startup, the
weighted spread of each sum-normalized term was:

| Term | Raw mean | Weighted spread after ÷sum |
|---|---|---|
| heading | ≈ 0.5 rad | **1.6 × 10⁻²** |
| clearance | ≈ 0.17 /m | 6 × 10⁻⁵ |
| velocity | ≈ 1.0 m/s | 9 × 10⁻⁵ |
| goalDistance | ≈ 27 m | 2 × 10⁻⁵ |

The goal-distance term has a large mean and a tiny spread, so dividing by its
sum dilutes it into irrelevance; the heading term has a small mean and a large
relative spread, so it survives and dominates by roughly two orders of
magnitude. The planner turns smartly to face the goal and then never
accelerates — observed peak speed 0.08 m/s against a 1.0 m/s limit, still 0.18
m from its start after 2000 cycles.

Min-max normalization removes the dependence on each term's mean magnitude,
which is what makes the weights portable across scenarios.

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
to the admissible window. An earlier version code indexed element `[1]` of the empty
result instead, which is undefined behaviour. The `trapped` scenario covers it.
