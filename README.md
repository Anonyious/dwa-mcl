# Robotics: local planning and localization

Two classical mobile-robotics algorithms in C++, each with its derivation, a
test suite, and measurements that back up every tuning choice.

| | What it does | Stack |
|---|---|---|
| [**dwa_local_planner**](dwa_local_planner/) | Dynamic Window Approach local planner | C++17, Eigen, OpenCV, CMake |
| [**mcl_localization**](mcl_localization/) | Monte Carlo Localization (particle filter) | C++17, ROS 2, tf2 |

<p align="center">
  <img src="dwa_local_planner/data/dwa_default.gif" width="46%" alt="DWA planner">
  <img src="mcl_localization/data/mcl_tracking.gif" width="46%" alt="Particle filter">
</p>

Both animations are rendered from the programs' own trace output, not from a
separate reimplementation.

## What is here

Each project is split so that the algorithm is independently testable:
`dwa_core` has no OpenCV dependency and `mcl_core` has no ROS dependency, so
every cost term, motion model and sensor model can be unit-tested directly, and
the ROS layer is a thin translation at the boundary.

That split is also what makes the measurements possible. `mcl_localization`
ships an offline evaluation harness instead of a recorded bag: it drives a
simulated robot through the map against known ground truth, so localization
error is a number rather than an impression from watching RViz.

```bash
mcl_evaluate --map map/map.yaml --mode track --seeds 8
#   8/8 seeds converged, mean final error 0.055 m, over a 20 m route
#   with odometry drifting 0.35-0.71 m
```

## Some things the measurements settled

**Normalizing DWA's cost terms by their sum does not work.** It preserves each
term's *relative* spread, so goal distance (mean ≈27 m, spread 0.16 m) is
diluted into irrelevance while heading error (mean ≈0.5 rad, spread 0.56 rad)
dominates by two orders of magnitude. The planner turns smartly to face the
goal and then never accelerates — peak speed 0.08 m/s against a 1.0 m/s limit,
0.18 m from its start after 2 000 cycles. Min-max normalization fixes it.
[Details.](dwa_local_planner/docs/cost_function.md)

**The textbook product over laser beams is worse than a geometric mean.** With
30 beams the log-likelihood gap between a good particle and a mediocre one runs
to the hundreds, so one particle takes all the weight and the first resample
annihilates the cloud. Dividing by the beam count gives 8/8 seeds converging at
5 000 particles where the raw product gives 6/8, and makes `beam_skip` a pure
compute knob rather than something that silently retunes the filter's
confidence. [Details.](mcl_localization/docs/algorithm.md)

**A likelihood field in cells² is not a likelihood field in metres².** Feeding
squared cell distance into a Gaussian evaluates `exp(−d⁴/2σ²)`, not
`exp(−d²/2σ²)`, which collapses the sensor model to nothing beyond about one
cell and forces the particle count up by an order of magnitude.

**Map symmetry decides whether global localization is possible at all.** The
shipped map is generated asymmetric on purpose — three differently proportioned
rooms, an alcove, two off-centre pillars — because a symmetric floorplan leaves
mirrored poses that explain a scan equally well.

## Reading guide

| Document | Contents |
|---|---|
| [dwa_local_planner/README.md](dwa_local_planner/README.md) | build, run, parameters, scenarios |
| [dwa_local_planner/docs/cost_function.md](dwa_local_planner/docs/cost_function.md) | cost-term derivation, normalization, weight sensitivity |
| [mcl_localization/README.md](mcl_localization/README.md) | build, run, topics, parameters |
| [mcl_localization/docs/algorithm.md](mcl_localization/docs/algorithm.md) | motion and sensor models, beam combination, measured limits |

Both write-ups state what does *not* work: the planner has no global planner
and will sit in a concave dead end, and the particle filter needs ~5 000
particles for global localization because KLD-adaptive sampling and random
particle injection are not implemented.

## Quick start

```bash
# DWA planner -- no ROS needed
cmake -S dwa_local_planner -B build/dwa -DCMAKE_BUILD_TYPE=Release
cmake --build build/dwa -j
./build/dwa/dwa_local_planner --help
cd build/dwa && ctest --output-on-failure

# MCL -- needs a ROS 2 workspace
colcon build --packages-select mcl_localization
ros2 launch mcl_localization localization.launch.py
colcon test --packages-select mcl_localization
```

Build `dwa_local_planner` with `-DDWA_WITH_GUI=OFF` to skip the OpenCV
visualization entirely; the planner and its tests then run fully headless.

## Author

**Abhinav Thakare** — `abhinavthakare99@gmail.com`

Licensed under [BSD-3-Clause](LICENSE).
