#include "mpc_planner_types/data_types.h"

#include <numeric>
#include <string>
#include <algorithm>

/** Basic high-level data types for motion planning */

namespace MPCPlanner
{

    Disc::Disc(const double offset_, const double radius_)
        : offset(offset_), radius(radius_)
    {
    }

    Eigen::Vector2d Disc::getPosition(const Eigen::Vector2d &robot_position, const double angle) const
    {
        return robot_position + Eigen::Vector2d(offset * std::cos(angle), offset * std::sin(angle));
    }

    Eigen::Vector2d Disc::toRobotCenter(const Eigen::Vector2d &disc_position, const double angle) const
    {
        return disc_position - Eigen::Vector2d(offset * std::cos(angle), offset * std::sin(angle));
    }

    Halfspace::Halfspace(const Eigen::Vector2d &A, const double b)
        : A(A), b(b)
    {
    }

    PredictionStep::PredictionStep(const Eigen::Vector2d &position, double angle, double major_radius, double minor_radius)
        : position(position), angle(angle), major_radius(major_radius), minor_radius(minor_radius)
    {
    }

    Prediction::Prediction()
        : type(PredictionType::NONE)
    {
    }

    Prediction::Prediction(PredictionType type)
        : type(type)
    {
        if (type == PredictionType::DETERMINISTIC || type == PredictionType::GAUSSIAN)
        {
            modes.emplace_back();
            probabilities.emplace_back(1.);
        }
    }

    bool Prediction::empty() const
    {
        return modes.empty() || (modes.size() > 0 && modes[0].empty());
    }

    std::string Prediction::toString() const
    {
        std::string type_str;
        switch (type)
        {
        case PredictionType::DETERMINISTIC:
            type_str = "DETERMINISTIC";
            break;
        case PredictionType::GAUSSIAN:
            type_str = "GAUSSIAN";
            break;
        case PredictionType::NONGAUSSIAN:
            type_str = "NONGAUSSIAN";
            break;
        case PredictionType::NONE:
            type_str = "NONE";
            break;
        }

        std::string result = "Prediction{type:" + type_str + ", modes:" + std::to_string(modes.size()) + "[";

        // Add mode information using accumulate pattern (limit to first 2 modes for brevity)
        for (size_t m = 0; m < modes.size() && m < 2; ++m)
        {
            if (m > 0)
                result += ", ";
            result += "mode" + std::to_string(m) + ":[";

            const auto &mode = modes[m];
            result += std::accumulate(mode.begin(),
                                      std::min(mode.end(), mode.begin() + 2), // Limit to first 2 steps
                                      std::string(),
                                      [](const std::string &s, const PredictionStep &step)
                                      {
                                          return s + (s.empty() ? "" : ",") + step.toString();
                                      });
            if (mode.size() > 2)
                result += ",...(+" + std::to_string(mode.size() - 2) + ")";
            result += "]";
        }
        if (modes.size() > 2)
            result += ",...(+" + std::to_string(modes.size() - 2) + ")";

        result += "], probs:[" +
                  std::accumulate(probabilities.begin(), probabilities.end(), std::string(),
                                  [](const std::string &s, double p)
                                  {
                                      return s + (s.empty() ? "" : ",") + std::to_string(p);
                                  }) +
                  "]}";
        return result;
    }

    DynamicObstacle::DynamicObstacle(int _index, const Eigen::Vector2d &_position, double _angle, double _radius, ObstacleType _type)
        : index(_index), position(_position), angle(_angle), radius(_radius)
    {
        type = _type;
    }

    DynamicObstacle::DynamicObstacle(int _index, double _radius, ObstacleType _type) : index(_index), radius(_radius)
    {
        type = _type;
    }

    void DynamicObstacle::updateDynamicObstacleState(const Eigen::Vector2d &_new_position, const double &_new_angle, Prediction _new_prediction)
    {
        position = _new_position;
        angle = _new_angle;
        prediction = _new_prediction; // I dont know if this is possible now, what kind of constructor does this call?
    }

    std::string DynamicObstacle::toString() const
    {
        std::string type_str;
        switch (type)
        {
        case ObstacleType::STATIC:
            type_str = "STATIC";
            break;
        case ObstacleType::DYNAMIC:
            type_str = "DYNAMIC";
            break;
        case ObstacleType::ROBOT:
            type_str = "ROBOT";
            break;
        }

        return "DynamicObstacle{id:" + std::to_string(index) +
               ", pos:[" + std::to_string(position.x()) + "," + std::to_string(position.y()) + "]" +
               ", angle:" + std::to_string(angle) +
               ", speed:" + std::to_string(current_speed) +
               ", radius:" + std::to_string(radius) +
               ", type:" + type_str +
               ", prediction:" + prediction.toString() + "}";
    }

    ReferencePath::ReferencePath(int length)
    {
        x.reserve(length);
        y.reserve(length);
        psi.reserve(length);
        v.reserve(length);
        s.reserve(length);
    }

    void ReferencePath::clear()
    {
        x.clear();
        y.clear();
        psi.clear();
        v.clear();
        s.clear();
    }

    bool ReferencePath::pointInPath(int point_num, double other_x, double other_y) const
    {
        return (x[point_num] == other_x && y[point_num] == other_y);
    }

    Trajectory::Trajectory(double dt, int length) : dt(dt)
    {
        positions.reserve(length);
        orientations.reserve(length);
    }

    void Trajectory::add(const Eigen::Vector2d &p)
    {
        positions.push_back(p);
    }

    void Trajectory::add(const double x, const double y)
    {
        positions.push_back(Eigen::Vector2d(x, y));
    }

    void Trajectory::add_orientation(const double psi)
    {
        orientations.push_back(psi);
    }

    std::string PredictionStep::toString() const
    {
        return "PredictionStep{pos:[" + std::to_string(position.x()) + "," + std::to_string(position.y()) +
               "], angle:" + std::to_string(angle) +
               ", maj_r:" + std::to_string(major_radius) +
               ", min_r:" + std::to_string(minor_radius) + "}";
    }

    // Jules Function to calculate the space time overlap between to trajectories, low values of result mean the smallest overlap sum.
    double Trajectory::calcCollisionMaskGK(const Trajectory &otherTraject, double sigma)
    {
        // Sigma can be set to around 2x robot radius
        const size_t n = positions.size();
        if (n != otherTraject.positions.size())
        {
            return 0.0; // or throw an error depending on your design
        }

        const double sigma2 = sigma * sigma;
        double result = 0.0;

        for (size_t k = 0; k < n; ++k)
        {
            const Eigen::Vector2d &ego_pos = positions[k];
            const Eigen::Vector2d &other_pos = otherTraject.positions[k];
            const double dist2 = (ego_pos - other_pos).squaredNorm();

            // Gaussian kernel contribution, weighted by Δt
            result += std::exp(-dist2 / sigma2) * dt;
        }

        return result;
    }

    FixedSizeTrajectory::FixedSizeTrajectory(int size)
        : _size(size)
    {
        positions.reserve(size);
    }

    FixedSizeTrajectory::FixedSizeTrajectory(MPCPlanner::Trajectory trajectory, int size)
        : _size(size)
    {
        positions.reserve(size);
        replaceTrajectory(trajectory);
    }

    void FixedSizeTrajectory::add(const Eigen::Vector2d &p)
    {
        // On jump, erase the trajectory
        if (std::sqrt((p - positions.back()).transpose() * (p - positions.back())) > 5.0)
        {
            positions.clear();
            positions.push_back(p);
            return;
        }

        if ((int)positions.size() < _size)
        {
            positions.push_back(p);
        }
        else
        {
            positions.erase(positions.begin());
            positions.push_back(p);
        }
    }

    void FixedSizeTrajectory::replaceTrajectory(const MPCPlanner::Trajectory trajectory){
        positions.clear();
        for(const auto p : trajectory.positions){
            positions.push_back(p);
        }
    }
}