#ifndef MPC_DATA_TYPES_H
#define MPC_DATA_TYPES_H

#include <Eigen/Dense>

#include <vector>

/** Basic high-level data types for motion planning */

namespace MPCPlanner
{

    struct Disc
    {
        double offset;
        double radius;

        Disc(const double offset_, const double radius_);

        Eigen::Vector2d getPosition(const Eigen::Vector2d &robot_position, const double angle) const;
        Eigen::Vector2d toRobotCenter(const Eigen::Vector2d &disc_position, const double angle) const;
    };

    struct Halfspace
    {
        // Ax <= b
        Eigen::Vector2d A;
        double b;

        Halfspace(const Eigen::Vector2d &A, const double b);
    };
    typedef std::vector<Halfspace> StaticObstacle; // For all k, a halfspace

    enum class PredictionType
    {
        DETERMINISTIC = 0,
        GAUSSIAN,
        NONGAUSSIAN,
        NONE
    };

    struct PredictionStep
    {

        // Mean
        Eigen::Vector2d position;
        double angle;

        // Covariance
        double major_radius;
        double minor_radius;

        PredictionStep(const Eigen::Vector2d &position, double angle, double major_radius, double minor_radius);

        // Debug function for logging
        std::string toString() const;
    };

    typedef std::vector<PredictionStep> Mode;

    struct Prediction
    {

        PredictionType type;

        // std::vector<PredictionStep> Mode
        std::vector<Mode> modes;
        std::vector<double> probabilities;

        Prediction();
        Prediction(PredictionType type);

        bool empty() const;

        // Debug function for logging
        std::string toString() const;
    };

    enum class ObstacleType
    {
        STATIC = 0,
        DYNAMIC,
        ROBOT // Jule: This one you added to make a differentiation between dynmic obstacles and other robots whihc are seen by the ego robot as dynamic obstacles
    };

    struct DynamicObstacle
    {
        int index;

        Eigen::Vector2d position;
        double angle;
        double current_speed;

        double radius;
        ObstacleType type{ObstacleType::DYNAMIC};

        Prediction prediction;

        DynamicObstacle(int _index, const Eigen::Vector2d &_position, double _angle, double _radius, ObstacleType _type = ObstacleType::DYNAMIC);
        DynamicObstacle(int _index, double _radius, ObstacleType _type = ObstacleType::DYNAMIC); // Jules deze heb jij toevegoed

        void updateDynamicObstacleState(const Eigen::Vector2d &_new_position, const double &_new_angle, Prediction _new_prediction);

        // Debug function for logging
        std::string toString() const;
    };

    struct ReferencePath
    {

        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> psi;

        std::vector<double> v;
        std::vector<double> s;

        ReferencePath(int length = 10);
        void clear();

        bool pointInPath(int point_num, double other_x, double other_y) const;

        bool empty() const { return x.empty(); }
        bool hasVelocity() const { return !v.empty(); }
        bool hasDistance() const { return !s.empty(); }
    };

    typedef ReferencePath Boundary;

    struct Trajectory
    {
        double dt;
        std::vector<Eigen::Vector2d> positions;
        std::vector<double> orientations; // JULES deze heb jij zelf toegevoegd om de orientatie van een positie in een traject te weten

        Trajectory(double dt = 0., int length = 10);

        void add(const Eigen::Vector2d &p);
        void add(const double x, const double y);
        void add_orientation(const double psi);                                   // JULES deze heb jij zelf toegevoegd om de orientatie van een positie in een traject te weten
        double calcCollisionMaskGK(const Trajectory &otherTraject, double sigma); // Calculate how much two trajectories overlap in space time
    };

    struct FixedSizeTrajectory
    {
    private:
        int _size;

    public:
        std::vector<Eigen::Vector2d> positions;

        FixedSizeTrajectory(int size = 50);

        void add(const Eigen::Vector2d &p);
    };

    enum class PlannerState
    {
        INITIALIZING = 0,    // Robot is starting up, waiting for sync
        WAITING_FOR_DATA,    // Waiting for essential data (state, goal, path)
        FIRST_TIME_PLANNING, // Special handling for first planning cycle
        NORMAL_OPERATION,    // Standard MPC planning and control
        GOAL_REACHED,        // Robot has reached its objective
        EMERGENCY_BRAKING,   // Solver failed or safety issue detected
        ERROR_STATE          // Unrecoverable error state
    };
}

#endif