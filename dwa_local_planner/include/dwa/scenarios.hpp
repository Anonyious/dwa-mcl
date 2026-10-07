#ifndef DWA_SCENARIOS_HPP_
#define DWA_SCENARIOS_HPP_

#include <optional>
#include <string>
#include <vector>

#include "dwa/types.hpp"

namespace dwa {

struct Scenario {
  std::string name;
  std::string description;
  State start = State::Zero();
  Eigen::Vector2d goal = Eigen::Vector2d::Zero();
  std::vector<Obstacle> obstacles;
};

// Returns nullopt for an unknown name.
std::optional<Scenario> makeScenario(const std::string& name);

std::vector<std::string> scenarioNames();

}  // namespace dwa

#endif  // DWA_SCENARIOS_HPP_
