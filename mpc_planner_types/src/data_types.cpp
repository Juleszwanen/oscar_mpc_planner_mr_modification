#include "mpc_planner_types/data_types.h"
#include <mpc_planner_types/multi_robot_utility_functions.h>

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
        last_trajectory_update_time = ros::Time::now(); 
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

    bool Trajectory::geomtricDeviationTrigger(const Trajectory &broadcasted_traject, double max_deviation) const
    {
        // Check if the sizes of the trajectories are equal
        if (positions.size() != broadcasted_traject.positions.size())
        {
            return false; // or throw an error - cannot compare trajectories of different sizes
        }

        const double max_deviation_squared = max_deviation * max_deviation;

        for (size_t i = 0; i < positions.size(); ++i)
        {
            const double dx_squared = std::pow(positions[i].x() - broadcasted_traject.positions[i].x(), 2);
            const double dy_squared = std::pow(positions[i].y() - broadcasted_traject.positions[i].y(), 2);

            if (dx_squared + dy_squared > max_deviation_squared)
            {
                return true;
            }
        }

        return false;
    }

    void Trajectory::interpolateTrajectoryByElapsedTime(const ros::Time current_time, size_t N, double control_frequency, double robot_max_velocity, double robot_max_angular_velocity)
    {
        // ============================================================
        // VALIDATION CHECKS
        // ============================================================
        if (positions.empty())
        {
            return;
        }

        size_t n_measured = positions.size();

        if (n_measured != N)
        {
            return; // Size mismatch - skip interpolation
        }

        // Check orientations size matches
        if (orientations.size() != n_measured)
        {
            return; // Orientation size mismatch
        }

        // ========== CALCULATE ELAPSED TIME ==========
        ros::Duration elapsed_time = current_time - last_trajectory_update_time;
        double dt_interp = elapsed_time.toSec();

        double min_age_threshold = 1.0 / control_frequency;
        if (dt_interp < min_age_threshold)
        {
            return; // Skip interpolation for fresh trajectories
        }

        // ========== COMPUTE TIME SHIFT PARAMETERS ==========
        int k = static_cast<int>(std::floor(dt_interp / dt));
        double tau = dt_interp - k * dt;
        double alpha = tau / dt; // Fractional remainder [0, 1)

        // Cap k to prevent going beyond trajectory
        if (k >= static_cast<int>(N))
        {
            return; // Trajectory too stale
        }

        // Skip if no meaningful shift
        if (k == 0 && alpha < 0.01)
        {
            return;
        }

        // ============================================================
        // STEP 1: EXTRAPOLATE POINTS FROM END OF TRAJECTORY
        // ============================================================
        std::vector<Eigen::Vector2d> extrapolated_positions;
        std::vector<double> extrapolated_orientations;
        int num_extrap_points = k + 1;

        if (n_measured >= 2)
        {
            const Eigen::Vector2d &x_last_pos = positions.back();
            const Eigen::Vector2d &x_second_last_pos = positions[n_measured - 2];

            const double &x_last_orientation = orientations.back();
            const double &x_second_last_orientation = orientations[n_measured - 2];

            // Compute velocity and angular velocity from last two points
            Eigen::Vector2d v = (x_last_pos - x_second_last_pos) / dt;
            double psi_dot = MultiRobot::wrapAngleDifference(x_last_orientation - x_second_last_orientation) / dt;

            // Clamp velocities to physical limits
            double v_mag = v.norm();
            if (v_mag > robot_max_velocity)
            {
                v = v.normalized() * robot_max_velocity;
            }

            if (std::abs(psi_dot) > robot_max_angular_velocity)
            {
                psi_dot = (psi_dot > 0) ? robot_max_angular_velocity : -robot_max_angular_velocity;
            }

            // Generate k + 1 extrapolated points
            for (int i = 1; i <= num_extrap_points; ++i)
            {
                double t_extrap = i * dt;

                Eigen::Vector2d pos_extrap = x_last_pos + v * t_extrap;
                double psi_extrap = MultiRobot::wrapAngle(x_last_orientation + psi_dot * t_extrap);

                extrapolated_positions.emplace_back(pos_extrap);
                extrapolated_orientations.emplace_back(psi_extrap);
            }
        }
        else
        {
            return; // Cannot extrapolate - insufficient points
        }

        // ============================================================
        // STEP 2: REMOVE FIRST k POINTS
        // ============================================================
        if (k > 0)
        {
            positions.erase(positions.begin(), positions.begin() + k);
            orientations.erase(orientations.begin(), orientations.begin() + k);
        }

        // ============================================================
        // STEP 3: APPEND EXTRAPOLATED POINTS
        // ============================================================
        positions.insert(positions.end(), extrapolated_positions.begin(), extrapolated_positions.end());
        orientations.insert(orientations.end(), extrapolated_orientations.begin(), extrapolated_orientations.end());

        // ============================================================
        // STEP 4: INTERPOLATE ALL POINTS BY alpha
        // ============================================================
        if (alpha > 0.001)
        {
            std::vector<Eigen::Vector2d> interpolated_positions;
            std::vector<double> interpolated_orientations;

            interpolated_positions.reserve(N);
            interpolated_orientations.reserve(N);

            // Interpolate between consecutive pairs
            // We have (N+1) points, so we get N interpolated points
            for (size_t i = 0; i < positions.size() - 1; ++i)
            {
                const Eigen::Vector2d &x_curr_position = positions[i];
                const Eigen::Vector2d &x_next_position = positions[i + 1];

                const double &x_curr_orientation = orientations[i];
                const double &x_next_orientation = orientations[i + 1];

                Eigen::Vector2d pos_interp = (1.0 - alpha) * x_curr_position + alpha * x_next_position;
                double psi_interp = MultiRobot::interpolateAngle(x_curr_orientation, x_next_orientation, alpha);

                interpolated_positions.emplace_back(pos_interp);
                interpolated_orientations.emplace_back(psi_interp);
            }

            positions = std::move(interpolated_positions);
            orientations = std::move(interpolated_orientations);
        }
        else
        {
            // No fractional shift needed, check if we have an extra point
            if (positions.size() > N)
            {
                positions.pop_back();
                orientations.pop_back();
            }
        }

        // ============================================================
        // STEP 5: VERIFY EXACTLY N POINTS
        // ============================================================
        while (positions.size() < N)
        {
            positions.push_back(positions.back());
            orientations.push_back(orientations.back());
        }
        while (positions.size() > N)
        {
            positions.pop_back();
            orientations.pop_back();
        }

        // ============================================================
        // UPDATE TIMESTAMP TO PREVENT COMPOUNDING DRIFT
        // ============================================================
        last_trajectory_update_time = current_time;
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