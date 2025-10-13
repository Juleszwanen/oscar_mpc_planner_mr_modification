#ifndef DATA_PREPARATION_H
#define DATA_PREPARATION_H

#include <Eigen/Dense>
#include <vector>
#include <set>

namespace MPCPlanner
{
  struct Disc;
  struct State;
  struct DynamicObstacle;
  struct Prediction;
  struct RealTimeData;
  
  

  std::vector<Disc> defineRobotArea(double length, double width, int n_discs);

  DynamicObstacle getDummyObstacle(const State &state);

  Prediction getConstantVelocityPrediction(const Eigen::Vector2d &position,
                                           const Eigen::Vector2d &velocity,
                                           double dt, int steps);

  void removeDistantObstacles(std::vector<DynamicObstacle> &obstacles, const State &state);
  void ensureObstacleSize(std::vector<DynamicObstacle> &obstacles, const State &state);

  void propagatePredictionUncertainty(Prediction &prediction);
  void propagatePredictionUncertainty(std::vector<DynamicObstacle> &obstacles);

  /** @note Jules: Functions from here on are added by you*/
  namespace MultiRobot
  {
    void updateRobotObstaclesFromTrajectories(RealTimeData &_data, const std::set<std::string> &_validated_trajectory_robots, const std::string &_ego_robot_ns);
  }
} // namespace MPCPlanner

#endif // DATA_PREPARATION_H
