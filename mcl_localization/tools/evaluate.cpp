// Offline evaluation harness for the particle filter.
//
// Loads the shipped map, drives a simulated robot through it, corrupts the
// odometry with drift, ray-casts laser scans against the map, runs the filter,
// and reports the tracking error. Links mcl_core only -- no ROS, so it builds
// and runs anywhere the library does.
//
// This exists in place of a recorded bag file: the scenario is reproducible
// from a seed, the ground truth is known (so the error is measurable rather
// than eyeballed in RViz), and there is no opaque binary blob in the repo.
//
//   mcl_evaluate --map map/map.yaml --mode track --particles 1000
//   mcl_evaluate --map map/map.yaml --mode global --seeds 8
//   mcl_evaluate --map map/map.yaml --trace run.csv

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "mcl/particle_filter.hpp"

namespace {

struct MapYaml {
  std::string image;
  double resolution = 0.05;
  double originX = 0.0;
  double originY = 0.0;
  double originYaw = 0.0;
  double occupiedThresh = 0.65;
  double freeThresh = 0.196;
  int negate = 0;
};

std::string directoryOf(const std::string& path) {
  const std::size_t slash = path.find_last_of("/\\");
  return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
}

std::string trim(const std::string& s) {
  const std::size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  const std::size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

bool loadMapYaml(const std::string& path, MapYaml& out) {
  std::ifstream in(path);
  if (!in) {
    std::cerr << "cannot open " << path << "\n";
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    const std::size_t hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    std::string value = trim(line.substr(colon + 1));
    if (key == "image") {
      out.image = value;
    } else if (key == "resolution") {
      out.resolution = std::atof(value.c_str());
    } else if (key == "occupied_thresh") {
      out.occupiedThresh = std::atof(value.c_str());
    } else if (key == "free_thresh") {
      out.freeThresh = std::atof(value.c_str());
    } else if (key == "negate") {
      out.negate = std::atoi(value.c_str());
    } else if (key == "origin") {
      // [x, y, yaw]
      for (char& c : value) {
        if (c == '[' || c == ']' || c == ',') {
          c = ' ';
        }
      }
      std::istringstream ss(value);
      ss >> out.originX >> out.originY >> out.originYaw;
    }
  }
  return !out.image.empty();
}

// Minimal binary PGM (P5) reader.
bool loadPgm(const std::string& path, int& width, int& height,
             std::vector<std::uint8_t>& pixels) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::cerr << "cannot open " << path << "\n";
    return false;
  }

  std::string magic;
  in >> magic;
  if (magic != "P5") {
    std::cerr << path << ": expected P5, got " << magic << "\n";
    return false;
  }

  // Header fields, skipping comments.
  int values[3] = {0, 0, 0};
  for (int i = 0; i < 3;) {
    const int c = in.peek();
    if (c == '#') {
      std::string discard;
      std::getline(in, discard);
      continue;
    }
    if (std::isspace(c)) {
      in.get();
      continue;
    }
    in >> values[i];
    ++i;
  }
  in.get();  // the single whitespace after maxval

  width = values[0];
  height = values[1];
  pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
  in.read(reinterpret_cast<char*>(pixels.data()),
          static_cast<std::streamsize>(pixels.size()));
  return in.gcount() == static_cast<std::streamsize>(pixels.size());
}

// map_server's pixel -> occupancy conversion, including the vertical flip:
// PGM row 0 is the top of the image, grid row 0 is y = 0 at the bottom.
mcl::OccupancyGrid toOccupancyGrid(const MapYaml& yaml, int width, int height,
                                   const std::vector<std::uint8_t>& pixels) {
  std::vector<std::int8_t> cells(static_cast<std::size_t>(width) *
                                 static_cast<std::size_t>(height));
  for (int y = 0; y < height; ++y) {
    const int srcRow = height - 1 - y;
    for (int x = 0; x < width; ++x) {
      const std::uint8_t px =
          pixels[static_cast<std::size_t>(srcRow) * static_cast<std::size_t>(width) +
                 static_cast<std::size_t>(x)];
      double p = static_cast<double>(px) / 255.0;
      p = yaml.negate ? p : 1.0 - p;

      std::int8_t value = mcl::OccupancyGrid::kUnknown;
      if (p > yaml.occupiedThresh) {
        value = mcl::OccupancyGrid::kOccupied;
      } else if (p < yaml.freeThresh) {
        value = mcl::OccupancyGrid::kFree;
      }
      cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x)] = value;
    }
  }
  return mcl::OccupancyGrid(width, height, yaml.resolution,
                            mcl::Pose2D(yaml.originX, yaml.originY, yaml.originYaw),
                            std::move(cells));
}

// --- Simulated robot -------------------------------------------------------

double raycast(const mcl::OccupancyGrid& map, double x, double y, double theta,
               double maxRange) {
  const double step = map.resolution() * 0.5;
  for (double d = step; d < maxRange; d += step) {
    int gx = 0;
    int gy = 0;
    if (!map.worldToGrid(x + d * std::cos(theta), y + d * std::sin(theta), gx, gy)) {
      return maxRange;
    }
    if (map.isOccupied(gx, gy)) {
      return d;
    }
  }
  return maxRange;
}

mcl::LaserScan simulateScan(const mcl::OccupancyGrid& map, const mcl::Pose2D& pose,
                            int beams, double maxRange, double noiseStdDev,
                            std::mt19937& rng) {
  std::normal_distribution<double> noise(0.0, noiseStdDev);

  mcl::LaserScan scan;
  scan.angleMin = -mcl::kPi;
  scan.angleIncrement = 2.0 * mcl::kPi / static_cast<double>(beams);
  scan.rangeMin = 0.05;
  scan.rangeMax = maxRange;
  scan.ranges.reserve(static_cast<std::size_t>(beams));

  for (int i = 0; i < beams; ++i) {
    const double angle = mcl::wrapToPi(pose.theta + scan.angleAt(static_cast<std::size_t>(i)));
    const double range = raycast(map, pose.x, pose.y, angle, maxRange);
    scan.ranges.push_back(static_cast<float>(
        range >= maxRange ? maxRange : std::max(scan.rangeMin, range + noise(rng))));
  }
  return scan;
}

// Shortest path through free cells, so the route is guaranteed traversable.
std::vector<mcl::Pose2D> planRoute(const mcl::OccupancyGrid& map, double clearance) {
  const int w = map.width();
  const int h = map.height();
  const int pad = std::max(1, static_cast<int>(clearance / map.resolution()));

  // Only accept cells with free space all around, so the robot does not graze
  // the walls and the scans stay informative.
  const auto roomy = [&](int x, int y) {
    for (int dy = -pad; dy <= pad; ++dy) {
      for (int dx = -pad; dx <= pad; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (!map.inBounds(nx, ny) || !map.isFree(nx, ny)) {
          return false;
        }
      }
    }
    return true;
  };

  std::vector<int> candidates;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (roomy(x, y)) {
        candidates.push_back(y * w + x);
      }
    }
  }
  if (candidates.size() < 2) {
    return {};
  }

  // Start at the lowest-then-leftmost roomy cell, finish at the farthest one
  // reachable from it.
  const int start = candidates.front();
  std::vector<int> prev(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -2);
  std::deque<int> queue{start};
  prev[static_cast<std::size_t>(start)] = -1;
  int farthest = start;

  while (!queue.empty()) {
    const int cur = queue.front();
    queue.pop_front();
    farthest = cur;
    const int cx = cur % w;
    const int cy = cur / w;
    const int dxs[4] = {1, -1, 0, 0};
    const int dys[4] = {0, 0, 1, -1};
    for (int k = 0; k < 4; ++k) {
      const int nx = cx + dxs[k];
      const int ny = cy + dys[k];
      if (!map.inBounds(nx, ny) || !roomy(nx, ny)) {
        continue;
      }
      const int idx = ny * w + nx;
      if (prev[static_cast<std::size_t>(idx)] != -2) {
        continue;
      }
      prev[static_cast<std::size_t>(idx)] = cur;
      queue.push_back(idx);
    }
  }

  std::vector<int> chain;
  for (int cur = farthest; cur != -1; cur = prev[static_cast<std::size_t>(cur)]) {
    chain.push_back(cur);
  }
  std::reverse(chain.begin(), chain.end());

  std::vector<mcl::Pose2D> route;
  route.reserve(chain.size());
  for (std::size_t i = 0; i < chain.size(); ++i) {
    double wx = 0.0;
    double wy = 0.0;
    map.gridToWorld(chain[i] % w, chain[i] / w, wx, wy);
    route.push_back(mcl::Pose2D(wx, wy, 0.0));
  }
  // Headings from the direction of travel, smoothed over a short lookahead so
  // the grid-aligned BFS path does not produce 90-degree heading flips.
  for (std::size_t i = 0; i < route.size(); ++i) {
    const std::size_t j = std::min(route.size() - 1, i + 8);
    if (j > i) {
      route[i].theta = std::atan2(route[j].y - route[i].y, route[j].x - route[i].x);
    } else if (i > 0) {
      route[i].theta = route[i - 1].theta;
    }
  }
  return route;
}

struct Result {
  double finalError = 0.0;
  double meanError = 0.0;
  double maxError = 0.0;
  double odometryDrift = 0.0;
  double pathLength = 0.0;
  int recoveries = 0;
  bool converged = false;
};

struct Options {
  std::string mapPath = "map/map.yaml";
  std::string mode = "track";
  std::string tracePath;
  int particles = 1000;
  int beams = 360;
  int beamSkip = 12;
  int seeds = 1;
  int firstSeed = 1;
  // Mirrors mcl::SensorModel::Params::sigmaHit so a run with no flags
  // exercises the shipped defaults.
  double sigmaHit = 0.1;
  double convergeWithin = 0.5;
  double scanNoise = 0.01;
  double maxRange = 12.0;
  bool quiet = false;
  bool normalizeByBeamCount = true;
};

Result runOnce(const mcl::OccupancyGrid& map,
               const std::shared_ptr<const mcl::OccupancyGrid>& mapPtr,
               const std::vector<mcl::Pose2D>& route, const Options& opt,
               unsigned seed, std::ofstream* trace) {
  std::mt19937 rng(seed);

  mcl::ParticleFilter::Params filterParams;
  filterParams.particleCount = opt.particles;
  filterParams.seed = seed;

  mcl::MotionModel::Params motionParams;
  mcl::SensorModel::Params sensorParams;
  sensorParams.sigmaHit = opt.sigmaHit;
  sensorParams.beamSkip = opt.beamSkip;
  sensorParams.normalizeByBeamCount = opt.normalizeByBeamCount;

  mcl::ParticleFilter filter(filterParams, motionParams, sensorParams);
  filter.setMap(mapPtr);

  if (opt.mode == "global") {
    filter.initializeGlobal();
  } else {
    filter.initializeAtPose(route.front(), 0.3, 0.1);
  }

  // Odometry integrates the true deltas with a multiplicative error and a
  // small rotational bias, which is how real wheel odometry drifts.
  std::normal_distribution<double> driftNoise(0.0, 0.03);
  mcl::Pose2D odom = mcl::Pose2D(0.0, 0.0, route.front().theta);

  Result result;
  double errorSum = 0.0;
  int samples = 0;

  for (std::size_t i = 1; i < route.size(); ++i) {
    const mcl::Pose2D& truth = route[i];
    const mcl::Pose2D& previous = route[i - 1];

    const double dx = truth.x - previous.x;
    const double dy = truth.y - previous.y;
    const double trueStep = std::sqrt(dx * dx + dy * dy);
    result.pathLength += trueStep;

    const double noisyStep = trueStep * (1.0 + driftNoise(rng));
    const double dTheta =
        mcl::angleDiff(truth.theta, previous.theta) * (1.0 + driftNoise(rng)) +
        0.004 * trueStep;
    odom.theta = mcl::wrapToPi(odom.theta + dTheta);
    odom.x += noisyStep * std::cos(odom.theta);
    odom.y += noisyStep * std::sin(odom.theta);

    const mcl::LaserScan scan =
        simulateScan(map, truth, opt.beams, opt.maxRange, opt.scanNoise, rng);
    filter.update(odom, scan);

    const mcl::PoseEstimate estimate = filter.estimate();
    if (!estimate.valid) {
      continue;
    }
    const double error =
        std::hypot(estimate.mean.x - truth.x, estimate.mean.y - truth.y);
    errorSum += error;
    ++samples;
    result.maxError = std::max(result.maxError, error);
    result.finalError = error;

    if (trace != nullptr) {
      *trace << "pose," << i << "," << truth.x << "," << truth.y << ","
             << truth.theta << "," << estimate.mean.x << "," << estimate.mean.y
             << "," << estimate.mean.theta << "," << error << ","
             << estimate.effectiveSampleSize << "\n";
      // A subsample of the cloud keeps the trace small enough to plot.
      const std::vector<mcl::Particle>& particles = filter.particles();
      const std::size_t stride = std::max<std::size_t>(1, particles.size() / 150);
      for (std::size_t p = 0; p < particles.size(); p += stride) {
        *trace << "particle," << i << "," << particles[p].pose.x << ","
               << particles[p].pose.y << "," << particles[p].pose.theta
               << ",,,,,\n";
      }
    }
  }

  result.odometryDrift =
      std::hypot(odom.x - route.back().x, odom.y - route.back().y);
  result.meanError = samples > 0 ? errorSum / samples : 0.0;
  result.recoveries = filter.recoveryCount();
  result.converged = result.finalError < opt.convergeWithin;
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto next = [&](double fallback) {
      return i + 1 < argc ? std::atof(argv[++i]) : fallback;
    };
    if (a == "--map" && i + 1 < argc) {
      opt.mapPath = argv[++i];
    } else if (a == "--mode" && i + 1 < argc) {
      opt.mode = argv[++i];
    } else if (a == "--trace" && i + 1 < argc) {
      opt.tracePath = argv[++i];
    } else if (a == "--particles") {
      opt.particles = static_cast<int>(next(1000));
    } else if (a == "--beams") {
      opt.beams = static_cast<int>(next(360));
    } else if (a == "--beam-skip") {
      opt.beamSkip = static_cast<int>(next(12));
    } else if (a == "--seeds") {
      opt.seeds = static_cast<int>(next(1));
    } else if (a == "--first-seed") {
      opt.firstSeed = static_cast<int>(next(1));
    } else if (a == "--sigma-hit") {
      opt.sigmaHit = next(0.2);
    } else if (a == "--no-normalize") {
      opt.normalizeByBeamCount = false;
    } else if (a == "--quiet") {
      opt.quiet = true;
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "  --map FILE        map YAML (default map/map.yaml)\n"
                << "  --mode track|global\n"
                << "  --particles N     particle count\n"
                << "  --beams N         simulated beams per scan\n"
                << "  --beam-skip N     use every Nth beam\n"
                << "  --sigma-hit M     likelihood field sigma, metres\n"
                << "  --seeds N         run N seeds and summarise\n"
                << "  --no-normalize    raw product instead of the geometric mean\n"
                << "  --trace FILE      write a CSV trace (first seed only)\n"
                << "  --quiet           summary only\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  MapYaml yaml;
  if (!loadMapYaml(opt.mapPath, yaml)) {
    return 1;
  }
  const std::string imagePath = directoryOf(opt.mapPath) + "/" + yaml.image;

  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> pixels;
  if (!loadPgm(imagePath, width, height, pixels)) {
    return 1;
  }

  auto map = std::make_shared<mcl::OccupancyGrid>(
      toOccupancyGrid(yaml, width, height, pixels));
  if (map->empty()) {
    std::cerr << "map failed to load\n";
    return 1;
  }

  const std::vector<mcl::Pose2D> route = planRoute(*map, 0.35);
  if (route.size() < 10) {
    std::cerr << "could not find a traversable route through the map\n";
    return 1;
  }

  if (!opt.quiet) {
    std::cout << "map        " << width << "x" << height << " at "
              << yaml.resolution << " m/cell\n"
              << "route      " << route.size() << " poses, start ("
              << std::fixed << std::setprecision(2) << route.front().x << ", "
              << route.front().y << ") -> (" << route.back().x << ", "
              << route.back().y << ")\n"
              << "mode       " << opt.mode << ", " << opt.particles
              << " particles, every " << opt.beamSkip << "th of " << opt.beams
              << " beams\n"
              << std::endl;
  }

  std::ofstream trace;
  if (!opt.tracePath.empty()) {
    trace.open(opt.tracePath);
    trace << "# resolution," << yaml.resolution << "\n";
    trace << "# origin," << yaml.originX << "," << yaml.originY << "\n";
    trace << "# size," << width << "," << height << "\n";
    trace << "# image," << yaml.image << "\n";
    trace << "kind,step,a,b,c,d,e,f,g,h\n";
  }

  int converged = 0;
  double sumFinal = 0.0;
  for (int s = 0; s < opt.seeds; ++s) {
    const unsigned seed = static_cast<unsigned>(opt.firstSeed + s);
    const Result r =
        runOnce(*map, map, route, opt, seed,
                (s == 0 && trace.is_open()) ? &trace : nullptr);
    converged += r.converged ? 1 : 0;
    sumFinal += r.finalError;

    if (!opt.quiet) {
      std::cout << "seed " << std::setw(3) << seed << "   final "
                << std::fixed << std::setprecision(3) << r.finalError
                << " m   mean " << r.meanError << " m   max " << r.maxError
                << " m   odom drift " << r.odometryDrift << " m"
                << (r.converged ? "   OK" : "   DIVERGED") << "\n";
    }
  }

  std::cout << "\npath length      " << std::fixed << std::setprecision(2)
            << runOnce(*map, map, route, opt, 999, nullptr).pathLength << " m\n"
            << "converged        " << converged << "/" << opt.seeds
            << " seeds within " << opt.convergeWithin << " m\n"
            << "mean final error " << std::setprecision(3)
            << (opt.seeds ? sumFinal / opt.seeds : 0.0) << " m" << std::endl;

  return converged == opt.seeds ? 0 : 1;
}
