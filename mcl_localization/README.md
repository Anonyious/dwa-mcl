# MCL Localization

**Monte Carlo Localization for ROS 2** — an odometry motion model, a
likelihood-field range-finder model, and adaptive low-variance resampling over
a known 2D occupancy grid.

[![CI](https://github.com/Anonyious/dwa-mcl/actions/workflows/ci.yml/badge.svg)](https://github.com/Anonyious/dwa-mcl/actions/workflows/ci.yml)
![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue)
![License](https://img.shields.io/badge/license-BSD--3--Clause-green)

![Particle filter tracking](data/mcl_tracking.gif)

*Green: ground truth. Red: the filter's estimate. Blue: the particle cloud.
Rendered from the evaluation harness output — see [`tools/`](tools/).*

The point of this implementation is that its tuning is measured rather than
asserted. Instead of a recorded bag and a screenshot of RViz, the package ships
an offline harness that drives a simulated robot through the map against known
ground truth, so every claim below is a number you can reproduce from a seed:

```
$ mcl_evaluate --map map/map.yaml --mode track --seeds 8
path length      20.25 m
converged        8/8 seeds within 0.50 m
mean final error 0.055 m
```

| | Result |
|---|---|
| Tracking accuracy | **0.055 m** mean final error, 8/8 seeds, against odometry drifting 0.31–0.95 m |
| Particles needed | **200** for tracking, **5 000** for global localization |
| Unit tests | **83** cases, GoogleTest, no ROS required |
| Core dependencies | **none** — `mcl_core` is plain C++17 |

## Contents

| Path | |
|---|---|
| [`include/mcl/`](include/mcl/) | the algorithm: grid, likelihood field, motion and sensor models, filter |
| [`src/`](src/) | implementations, plus the ROS node |
| [`test/`](test/) | 83 GoogleTest cases |
| [`tools/`](tools/) | offline evaluation harness, map generator, GIF renderer |
| [`docs/algorithm.md`](docs/algorithm.md) | the derivations and every measurement behind the tuning |

```
include/mcl/   angles.hpp            wrapping, circular mean
               types.hpp             Pose2D, Particle, LaserScan, PoseEstimate
               occupancy_grid.hpp    origin-aware, bounds-checked conversion
               likelihood_field.hpp  O(n) exact distance transform
               motion_model.hpp      sample_motion_model_odometry
               sensor_model.hpp      likelihood_field_range_finder_model
               particle_filter.hpp   the filter
               mcl_node.hpp          the only header that includes rclcpp
```

`mcl_core` has **no ROS dependency**. The node translates messages into plain
structs at the boundary, which is what makes the models unit-testable and the
harness possible — the tests and `mcl_evaluate` build and run with nothing but
a C++17 compiler.

References: Thrun, Burgard & Fox, *Probabilistic Robotics* — Table 5.6
(motion), Table 6.3 (likelihood field), Table 4.4 (resampling), ch. 8 (MCL).

## Quick start

### With ROS 2

```bash
# from the workspace root, with this package under src/
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mcl_localization
source install/setup.bash

ros2 launch mcl_localization localization.launch.py
```

Then set an initial pose with **2D Pose Estimate** in RViz. The launch file
starts `nav2_map_server` (with a lifecycle manager, which it needs to reach
`active`), the `base_link`→`base_laser` static transform, the filter, and RViz.
Tested against the Humble/Jazzy API; CI runs on Jazzy.

| Launch argument | Default | Meaning |
|---|---|---|
| `map` | `map/map.yaml` | map to serve |
| `params_file` | `config/mcl.yaml` | parameter file |
| `use_sim_time` | `false` | set `true` when replaying a bag with `--clock` |
| `rviz` | `true` | start RViz |
| `initialize_globally` | `false` | start from a uniform cloud instead of waiting for `/initialpose` |

### Without ROS

The algorithm, the tests and the harness need only CMake and a C++17 compiler.
List `src/{occupancy_grid,likelihood_field,motion_model,sensor_model,particle_filter}.cpp`
plus `tools/evaluate.cpp` in a standalone `CMakeLists.txt` and everything
measurable in this README builds — which is how the numbers here were produced.

## Interface

| | Topic | Type |
|---|---|---|
| sub | `map` | `nav_msgs/OccupancyGrid` (transient-local) |
| sub | `scan` | `sensor_msgs/LaserScan` |
| sub | `odom` | `nav_msgs/Odometry` |
| sub | `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` |
| pub | `particlecloud` | `geometry_msgs/PoseArray` |
| pub | `mcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| TF | `map` → `odom` | |

The node publishes `map`→`odom`, per ROS convention; the odometry source owns
`odom`→`base_link`. The laser offset is read from the `base_frame`→scan-frame
transform, so it only has to be correct in one place.

## Parameters

All of them live in [`config/mcl.yaml`](config/mcl.yaml) and are declared ROS
parameters. Highlights:

| Parameter | Default | Notes |
|---|---|---|
| `particle_count` | 1000 | 200 suffices for tracking; global wants ~5000 |
| `beam_skip` | 12 | use every Nth beam; a pure compute knob |
| `sigma_hit` | 0.1 m | measured optimum; 0.35 and 0.5 are 3–4× worse |
| `z_hit` / `z_rand` | 0.9 / 0.1 | `z_rand` must be non-zero — it is the uniform floor |
| `alpha1`…`alpha4` | 0.05 | motion-model noise |
| `resample_threshold_ratio` | 0.5 | resample when `N_eff < ratio × N` |
| `normalize_by_beam_count` | `true` | geometric mean, not raw product |
| `random_seed` | 42 | runs are reproducible on purpose |

## Measured performance

Shipped map, 20.25 m route, 30 of 360 beams, odometry drifting 0.31–0.95 m
across the seeds. 8 seeds per row, mean final error in metres.

| | 200 | 500 | 1 000 | 2 000 | 5 000 particles |
|---|---|---|---|---|---|
| **tracking** (seeded prior) | 8/8, 0.063 | 8/8, 0.062 | 8/8, 0.055 | – | – |
| **global** (uniform cloud) | – | 1/8 | 2/8 | 4/8 | **8/8, 0.121** |

Tracking is reliable from 200 particles. Global localization needs about 5 000
to be dependable.

`sigma_hit` at 1 000 particles, tracking, same 8 seeds — the sensor model's
width is the single most sensitive parameter here:

| `sigma_hit` | 0.05 | **0.10** | 0.20 | 0.35 | 0.50 |
|---|---|---|---|---|---|
| mean final error | 0.067 m | **0.055 m** | 0.089 m | 0.160 m | 0.223 m |

### Two things the measurements settled

**The textbook product over laser beams is worse than a geometric mean.** With
30 beams the log-likelihood gap between a good particle and a mediocre one runs
to about 120, so one particle takes all the weight and the first resample
annihilates the cloud. Dividing by the beam count gives 8/8 seeds converging at
5 000 particles where the raw product gives 5/8 (mean final error 0.121 m
against 4.158 m), and it makes `beam_skip` a pure compute knob rather than
something that silently retunes the filter's confidence.

**A likelihood field in cells² is not a likelihood field in metres².** Feeding
squared cell distance into the Gaussian evaluates `exp(−d⁴/2σ²)`, not
`exp(−d²/2σ²)`, which collapses the sensor model to nothing beyond about one
cell and forces the particle count up by an order of magnitude.

Both are derived in [docs/algorithm.md](docs/algorithm.md).

## Reproducing the numbers

```bash
# the table above, row by row
mcl_evaluate --map map/map.yaml --mode track  --particles 200  --seeds 8
mcl_evaluate --map map/map.yaml --mode global --particles 5000 --seeds 8

# the geometric-mean comparison
mcl_evaluate --map map/map.yaml --mode global --particles 5000 --seeds 8 --no-normalize

# animate a run
mcl_evaluate --map map/map.yaml --trace run.csv
python tools/render_gif.py run.csv --map map/map.pgm -o data/mcl_tracking.gif
```

| Option | |
|---|---|
| `--mode track` / `--mode global` | seeded prior, or a uniform cloud over the free space |
| `--particles N`, `--beams N`, `--beam-skip N` | filter and scan sizing |
| `--sigma-hit M` | sensor-model width, metres |
| `--seeds N`, `--first-seed N` | run N seeds and summarise |
| `--no-normalize` | raw product instead of the geometric mean |
| `--trace FILE` | CSV trace of the first seed, for plotting |
| `--quiet` | summary only |

The harness exits 0 only when every seed converged, so it works as a CI gate on
localization accuracy rather than just on the build — which is how it is used
in [`.github/workflows/ci.yml`](../.github/workflows/ci.yml).

## The map

`map/map.pgm` is generated, not recorded:

```bash
python tools/make_map.py --show
```

The layout is deliberately asymmetric — three rooms of different proportions,
an alcove, two off-centre pillars. A symmetric floorplan leaves mirrored poses
that explain a scan equally well, and global localization can lock onto the
wrong one and never recover. The generator refuses to emit a map whose free
space is not a single connected component, and CI checks that the committed map
is byte-identical to what the generator produces.

## Tests

```bash
colcon test --packages-select mcl_localization
colcon test-result --verbose
```

83 cases covering angle wrapping and the circular mean, the rigid-transform
sensor offset, grid conversion (map origin, flooring, bounds, rotated origin),
the distance transform against brute force, motion-model σ non-negativity and
angle wrapping, the sensor model across scan lengths and beam counts, the
resampler's index bounds under degenerate weights, and end-to-end tracking
through drifting odometry.

| Suite | Cases |
|---|---|
| `test_particle_filter.cpp` | 26 |
| `test_angles.cpp` | 13 |
| `test_motion_model.cpp` | 12 |
| `test_sensor_model.cpp` | 12 |
| `test_occupancy_grid.cpp` | 11 |
| `test_likelihood_field.cpp` | 9 |

## Known limitations

- **Global localization needs ~5 000 particles**, against ~200 for tracking,
  and is unreliable at 2 000 and below (4/8 seeds at 2 000, 1/8 at 500).
  KLD-adaptive sampling would close that gap automatically but is **not
  implemented**, nor is random particle injection (AMCL's
  `recovery_alpha_slow/fast`), so the filter cannot recover if it does lock
  onto a wrong mode. See
  [docs/algorithm.md](docs/algorithm.md#what-is-not-implemented).
- **The reported pose is the mean over all particles**, not the dominant
  cluster's, so it is meaningless while the cloud is multi-modal. The published
  covariance is what makes that detectable — check it rather than trusting the
  mean.
- **No scan-matching or motion-model adaptation**, so very fast rotation or
  wheel slip beyond the `alpha` noise model will lose the lock.
- **Measurement updates run on the latest scan at a fixed rate**, without
  interpolating odometry to the scan timestamp. Fine at walking speed; a source
  of bias at speed.
- **Single-threaded**, O(particles × beams) per update.

## Author

**Abhinav Thakare** — `abhinavthakare99@gmail.com`

A correctness audit and rewrite: the algorithmic fixes are recorded in
[docs/algorithm.md](docs/algorithm.md) and in the git history, the algorithm
layer was separated from ROS, the package was ported to ROS 2, and the test
suite is new.

Part of [dwa-mcl](https://github.com/Anonyious/dwa-mcl), alongside a
[DWA local planner](../dwa_local_planner/).
Licensed under [BSD-3-Clause](../LICENSE).
