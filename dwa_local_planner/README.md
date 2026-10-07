# DWA Local Planner

A Dynamic Window Approach local planner for a differential-drive robot, with
an OpenCV visualization.

![Dynamic Window Approach](data/dwa_default.gif)

*Red: the trajectory the planner chose this cycle. Grey: the path travelled.
Shaded rings: the collision radius around each obstacle. Rendered from the
planner's own `--trace` output -- see `tools/`.*

At each control cycle the planner samples the velocity pairs reachable within
one timestep under its acceleration limits, forward-simulates each as a
constant control over a 4-second horizon, scores the resulting trajectories on
goal distance, heading error, obstacle clearance and speed, and executes the
best collision-free one for a single timestep.

Reference: Fox, Burgard & Thrun, "The Dynamic Window Approach to Collision
Avoidance", *IEEE Robotics & Automation Magazine* 4(1), 1997.
The cost function and its normalization are derived in
[docs/cost_function.md](docs/cost_function.md).

## Layout

```
include/dwa/           angles.hpp      angle wrapping
                       types.hpp       State, Control, PlannerConfig
                       planner.hpp     DwaPlanner  -- the algorithm
                       scenarios.hpp   built-in obstacle fields
                       visualizer.hpp  OpenCV rendering
src/                   implementations + main.cpp
tests/                 GoogleTest unit tests
```

`dwa_core` (the planner and scenarios) has no OpenCV or I/O dependency, which
is what makes every cost term directly unit-testable. Rendering lives in a
separate `dwa_viz` target.

## Dependencies

| | Version | Notes |
|---|---|---|
| CMake | ≥ 3.16 | |
| C++ | 17 | `std::optional`, `std::clamp` |
| Eigen | ≥ 3.3 | found via `Eigen3::Eigen` |
| OpenCV | ≥ 4.0 | `core`, `imgproc`, `highgui` |
| GoogleTest | 1.14 | auto-fetched if not installed |

Ubuntu/Debian:

```bash
sudo apt install build-essential cmake libeigen3-dev libopencv-dev
```

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Warnings are on by default (`-Wall -Wextra -Wpedantic`, or `/W4` on MSVC).

## Run

```bash
./build/dwa_local_planner                      # default scenario, with GUI
./build/dwa_local_planner --help               # all options
```

| Option | Meaning |
|---|---|
| `--scenario NAME` | obstacle field: `default`, `offset`, `trapped`, `corridor` |
| `--start X Y YAW_DEG` | override the start pose |
| `--goal X Y` | override the goal |
| `--robot-radius R` | collision footprint, metres |
| `--goal-tolerance R` | distance at which the goal counts as reached |
| `--max-iterations N` | abort after N control cycles |
| `--no-gui` | run headless, print progress |
| `--trace FILE` | write the run to CSV, for plotting |

Exit code is `0` if the goal was reached, `1` if the iteration limit was hit,
`2` on bad usage — so runs can be scripted. Press `ESC` to quit the GUI.

### Scenarios

| Name | What it is for |
|---|---|
| `default` | The obstacle field this planner was first built against. |
| `offset` | Start and goal both far from the world origin. Regression case for the heading cost, which used to be measured from the origin. |
| `trapped` | Robot starts inside an obstacle's collision radius, so every rollout collides and `plan()` has nothing to return. Exercises the recovery path. |
| `corridor` | Dead-end corridor sealed by a wall; the goal is unreachable. Checks that a thin obstacle is never tunneled through. |

## Parameters

Defaults live in `PlannerConfig` (`include/dwa/types.hpp`).

| Parameter | Default | Units |
|---|---|---|
| `maxLinearVelocity` | 1.0 | m/s |
| `minLinearVelocity` | −0.5 | m/s (reverse allowed) |
| `maxAngularVelocity` | 40 | deg/s |
| `maxLinearAcceleration` | 0.2 | m/s² |
| `maxAngularAcceleration` | 40 | deg/s² |
| `dt` | 0.1 | s |
| `predictionHorizon` | 4.0 | s |
| `linearVelocitySamples` | 11 | — |
| `angularVelocitySamples` | 21 | — |
| `goalDistanceWeight` | 1.0 | — |
| `goalHeadingWeight` | 1.0 | — |
| `obstacleWeight` | 1.0 | — |
| `velocityWeight` | 1.0 | — |
| `robotRadius` | 1.0 | m |
| `obstacleRadius` | 0.0 | m |
| `goalTolerance` | 1.0 | m |

The four cost weights are **not** independently tunable without checking
behaviour: over-weighting heading or clearance relative to velocity makes the
robot point at the goal and then never accelerate. See
[docs/cost_function.md](docs/cost_function.md#weights).

## Animating a run

The planner writes its own trace, so the GIF is this program's output rather
than a separate reimplementation:

```bash
./build/dwa_local_planner --scenario default --no-gui --trace run.csv
python tools/render_gif.py run.csv -o data/dwa_default.gif
```

Needs Pillow; no OpenCV and no display required.

## Tests

```bash
cd build && ctest --output-on-failure
```

Coverage is aimed at the pure functions, which is where the real bugs were:
angle wrapping across multiple revolutions, the dynamic-window derivation and
its clamping, each cost term (including translation-invariance of the heading
cost and NaN-freedom at degenerate goals), the goal test against a non-unit
tolerance, collision detection on odd-indexed poses, the recovery control, and
end-to-end convergence plus a never-enters-an-obstacle invariant on each
scenario.

## Known limitations

- **Obstacles are a ground-truth list, not sensed.** There is no sensor model
  and no local costmap, so this is a planner demo rather than something that
  could drive a real robot.
- **No global planner.** It is a pure local planner, so it will happily drive
  into a concave dead end and sit there — see the `corridor` scenario. A
  global-path alignment cost term is the usual fix; the hook for it was
  declared but never implemented, and has been removed rather
  than left as dead API.
- **Brute-force collision checking**, O(poses × obstacles) per candidate. Fine
  for tens of obstacles; a spatial index would be needed for a real costmap.
- **Rendering is single-window and blocking**; no headless image export.
- The circular-footprint assumption means in-place rotation is always
  collision-free, so a robot whose *start* pose is clear can never report
  "no valid trajectory" — only one already in collision can.

## Author

**Abhinav Thakare** — `abhinavthakare99@gmail.com`

