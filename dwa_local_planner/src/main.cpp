#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "dwa/planner.hpp"
#include "dwa/scenarios.hpp"
#ifdef DWA_WITH_GUI
#include "dwa/visualizer.hpp"
#endif

namespace {

struct CliOptions {
  std::string scenario = "default";
  std::optional<dwa::State> start;
  std::optional<Eigen::Vector2d> goal;
  std::optional<double> robotRadius;
  std::optional<double> goalTolerance;
  std::optional<int> maxIterations;
  std::string tracePath;
  bool gui = true;
  bool help = false;
};

void printUsage(const char* program) {
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Options:\n"
      << "  --scenario NAME        Obstacle field to run (default: default)\n"
      << "  --start X Y YAW_DEG    Override the start pose\n"
      << "  --goal X Y             Override the goal position\n"
      << "  --robot-radius R       Collision footprint radius, metres\n"
      << "  --goal-tolerance R     Distance at which the goal counts as reached\n"
      << "  --max-iterations N     Abort after N control steps\n"
      << "  --no-gui               Run headless (for CI and scripted checks)\n"
      << "  --trace FILE           Write the run to CSV, for plotting\n"
      << "  --help                 Show this message\n\n"
      << "Scenarios:\n";
  for (const std::string& name : dwa::scenarioNames()) {
    const std::optional<dwa::Scenario> scenario = dwa::makeScenario(name);
    std::cout << "  " << std::left << std::setw(10) << name << " "
              << (scenario ? scenario->description : "") << "\n";
  }
  std::cout << "\nExit codes: 0 goal reached, 1 iteration limit hit, 2 bad usage.\n";
}

// Reads `count` doubles following argv[i], advancing i past them.
bool readDoubles(int argc, char** argv, int& i, int count, std::vector<double>& out) {
  out.clear();
  for (int k = 0; k < count; ++k) {
    if (i + 1 >= argc) {
      return false;
    }
    try {
      out.push_back(std::stod(argv[++i]));
    } catch (const std::exception&) {
      return false;
    }
  }
  return true;
}

std::optional<CliOptions> parseArgs(int argc, char** argv) {
  CliOptions options;
  std::vector<double> values;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      options.help = true;
      return options;
    }
    if (arg == "--no-gui") {
      options.gui = false;
    } else if (arg == "--scenario") {
      if (i + 1 >= argc) {
        std::cerr << "--scenario needs a name\n";
        return std::nullopt;
      }
      options.scenario = argv[++i];
    } else if (arg == "--start") {
      if (!readDoubles(argc, argv, i, 3, values)) {
        std::cerr << "--start needs X Y YAW_DEG\n";
        return std::nullopt;
      }
      dwa::State start = dwa::State::Zero();
      start(dwa::kX) = values[0];
      start(dwa::kY) = values[1];
      start(dwa::kTheta) = dwa::degToRad(values[2]);
      options.start = start;
    } else if (arg == "--goal") {
      if (!readDoubles(argc, argv, i, 2, values)) {
        std::cerr << "--goal needs X Y\n";
        return std::nullopt;
      }
      options.goal = Eigen::Vector2d(values[0], values[1]);
    } else if (arg == "--robot-radius") {
      if (!readDoubles(argc, argv, i, 1, values)) {
        std::cerr << "--robot-radius needs a value\n";
        return std::nullopt;
      }
      options.robotRadius = values[0];
    } else if (arg == "--goal-tolerance") {
      if (!readDoubles(argc, argv, i, 1, values)) {
        std::cerr << "--goal-tolerance needs a value\n";
        return std::nullopt;
      }
      options.goalTolerance = values[0];
    } else if (arg == "--trace") {
      if (i + 1 >= argc) {
        std::cerr << "--trace needs a file path\n";
        return std::nullopt;
      }
      options.tracePath = argv[++i];
    } else if (arg == "--max-iterations") {
      if (!readDoubles(argc, argv, i, 1, values)) {
        std::cerr << "--max-iterations needs a value\n";
        return std::nullopt;
      }
      options.maxIterations = static_cast<int>(values[0]);
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return std::nullopt;
    }
  }
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  const std::optional<CliOptions> parsed = parseArgs(argc, argv);
  if (!parsed) {
    std::cerr << "Run with --help for usage.\n";
    return 2;
  }
  if (parsed->help) {
    printUsage(argv[0]);
    return 0;
  }

  const std::optional<dwa::Scenario> loaded = dwa::makeScenario(parsed->scenario);
  if (!loaded) {
    std::cerr << "Unknown scenario: " << parsed->scenario << "\n";
    return 2;
  }

  // Scenario provides the defaults; the command line overrides them. Unlike
  // an earlier version, the values passed in actually take effect.
  const dwa::State start = parsed->start.value_or(loaded->start);
  const Eigen::Vector2d goal = parsed->goal.value_or(loaded->goal);

  dwa::PlannerConfig config;
  if (parsed->robotRadius) {
    config.robotRadius = *parsed->robotRadius;
  }
  if (parsed->goalTolerance) {
    config.goalTolerance = *parsed->goalTolerance;
  }
  if (parsed->maxIterations) {
    config.maxIterations = *parsed->maxIterations;
  }

  const dwa::DwaPlanner planner(config, loaded->obstacles);

  std::cout << "scenario        " << loaded->name << "\n"
            << "start           (" << start(dwa::kX) << ", " << start(dwa::kY)
            << ") yaw " << start(dwa::kTheta) << " rad\n"
            << "goal            (" << goal.x() << ", " << goal.y() << ")\n"
            << "robot radius    " << config.robotRadius << " m\n"
            << "goal tolerance  " << config.goalTolerance << " m\n"
            << "obstacles       " << loaded->obstacles.size() << "\n"
            << std::endl;

#ifdef DWA_WITH_GUI
  std::unique_ptr<dwa::Visualizer> visualizer;
  if (parsed->gui) {
    dwa::Visualizer::Options vizOptions;
    visualizer = std::make_unique<dwa::Visualizer>(
        vizOptions, loaded->obstacles, Eigen::Vector2d(start(dwa::kX), start(dwa::kY)),
        goal, config.robotRadius, config.robotRadius + config.obstacleRadius);
  }
#else
  // Built without the OpenCV visualization; every run is headless.
  if (parsed->gui) {
    std::cout << "note: built with DWA_WITH_GUI=OFF, running headless"
              << std::endl;
  }
#endif

  // Optional CSV trace. Keeping this in the planner rather than
  // reimplementing the loop elsewhere means anything plotted from it is this
  // program's actual output.
  std::ofstream trace;
  if (!parsed->tracePath.empty()) {
    trace.open(parsed->tracePath);
    if (!trace) {
      std::cerr << "cannot write trace to " << parsed->tracePath << "\n";
      return 2;
    }
    trace << "# scenario," << loaded->name << "\n";
    trace << "# robot_radius," << config.robotRadius << "\n";
    trace << "# collision_radius," << config.robotRadius + config.obstacleRadius
          << "\n";
    trace << "# goal," << goal.x() << "," << goal.y() << "\n";
    trace << "# goal_tolerance," << config.goalTolerance << "\n";
    for (const dwa::Obstacle& obstacle : loaded->obstacles) {
      trace << "# obstacle," << obstacle.x << "," << obstacle.y << "\n";
    }
    trace << "kind,step,index,x,y,theta,v,w,recovery\n";
  }

  dwa::State state = start;
  dwa::Trajectory trajectory;
  bool reached = false;
  bool aborted = false;
  int recoveries = 0;
  int iteration = 0;

  for (; iteration < config.maxIterations; ++iteration) {
    if (planner.atGoal(state, goal)) {
      reached = true;
      break;
    }

    dwa::Control control;
    bool recovering = false;

    if (const std::optional<dwa::PlanResult> result = planner.plan(state, goal)) {
      control = result->control;
      trajectory = result->trajectory;
    } else {
      // No collision-free trajectory exists. An earlier version indexed into the
      // empty result here; instead, decelerate and rotate in place.
      control = planner.recoveryControl(state);
      trajectory.clear();
      recovering = true;
      ++recoveries;
    }

    dwa::DwaPlanner::step(control, state, config.dt);

    if (trace) {
      trace << "pose," << iteration << ",0," << state(dwa::kX) << ","
            << state(dwa::kY) << "," << state(dwa::kTheta) << "," << control(0)
            << "," << control(1) << "," << (recovering ? 1 : 0) << "\n";
      for (std::size_t t = 0; t < trajectory.size(); ++t) {
        trace << "traj," << iteration << "," << t << "," << trajectory[t](dwa::kX)
              << "," << trajectory[t](dwa::kY) << "," << trajectory[t](dwa::kTheta)
              << ",,," << (recovering ? 1 : 0) << "\n";
      }
    }

#ifdef DWA_WITH_GUI
    if (visualizer) {
      visualizer->render(state, trajectory, loaded->obstacles, goal, recovering);
      if (visualizer->show(5) == 27) {  // ESC
        aborted = true;
        break;
      }
    } else
#endif
        if (iteration % 50 == 0) {
      std::cout << "step " << std::setw(5) << iteration << "  pose ("
                << std::fixed << std::setprecision(2) << state(dwa::kX) << ", "
                << state(dwa::kY) << ")  v " << control(0) << "  w " << control(1)
                << (recovering ? "  [recovery]" : "") << std::endl;
    }
  }

  std::cout << "\n"
            << "result          "
            << (reached ? "goal reached" : (aborted ? "aborted by user" : "iteration limit"))
            << "\n"
            << "steps           " << iteration << "\n"
            << "recovery steps  " << recoveries << "\n"
            << "final pose      (" << std::fixed << std::setprecision(3)
            << state(dwa::kX) << ", " << state(dwa::kY) << ")\n"
            << "final distance  "
            << std::hypot(state(dwa::kX) - goal.x(), state(dwa::kY) - goal.y()) << " m"
            << std::endl;

#ifdef DWA_WITH_GUI
  if (visualizer && reached) {
    visualizer->show(0);  // hold the final frame until a key is pressed
  }
#endif

  if (aborted) {
    return 0;
  }
  return reached ? 0 : 1;
}
