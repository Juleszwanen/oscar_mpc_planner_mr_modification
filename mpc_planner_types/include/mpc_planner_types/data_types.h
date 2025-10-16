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

        FixedSizeTrajectory(int size = 30);
        FixedSizeTrajectory(MPCPlanner::Trajectory trajectory, int size = 30);

        void add(const Eigen::Vector2d &p);
        void replaceTrajectory(const MPCPlanner::Trajectory trajectory);
    };

    enum class PlannerState
    {
        UNINITIALIZED,                        // Just constructed, waiting for essential data
        TIMER_STARTUP,                        // Wait with doing anything before the timer goes of
        WAITING_FOR_FIRST_EGO_POSE,           // Wait for gazebo/optitrack/localization_algo to provide us with the first ego pose data
        INITIALIZING_OBSTACLES,               // Initializing other robots as obstacles, Setting up robot-robot obstacle tracking
        WAITING_FOR_OTHER_ROBOTS_FIRST_POSES, // Wait for the other robots to have send their first poses to the ego_robot
        WAITING_FOR_SYNC,                     // Waiting for other robots (if enabled)
        WAITING_FOR_TRAJECTORY_DATA,          // Have structure, waiting for first valid trajectories
        PLANNING_ACTIVE,                      // Normal MPC planning operation
        JUST_REACHED_GOAL,                    // JUST REACHED OUR GOAL, READY TO BREAK
        GOAL_REACHED,                         // At goal, may rotate or stop
        RESETTING,                            // Transitioning to new task
        ERROR_STATE                           // Unrecoverable error occurred
    };

    // Convert PlannerState to string for logging
    inline std::string stateToString(PlannerState state)
    {
        switch (state)
        {
        case PlannerState::UNINITIALIZED:
            return "UNINITIALIZED";
        case PlannerState::TIMER_STARTUP:
            return "TIMER_STARTUP";
        case PlannerState::WAITING_FOR_FIRST_EGO_POSE:
            return "WAITING_FOR_FIRST_EGO_POSE";
        case PlannerState::INITIALIZING_OBSTACLES:
            return "INITIALIZING_OBSTACLES";
        case PlannerState::WAITING_FOR_OTHER_ROBOTS_FIRST_POSES:
            return "WAITING_FOR_OTHER_ROBOTS_FIRST_POSES";
        case PlannerState::WAITING_FOR_SYNC:
            return "WAITING_FOR_SYNC";
        case PlannerState::WAITING_FOR_TRAJECTORY_DATA:
            return "WAITING_FOR_TRAJECTORY_DATA";
        case PlannerState::PLANNING_ACTIVE:
            return "PLANNING_ACTIVE";
        case PlannerState::JUST_REACHED_GOAL:
            return "JUST_REACHED_GOAL";
        case PlannerState::GOAL_REACHED:
            return "GOAL_REACHED";
        case PlannerState::RESETTING:
            return "RESETTING";
        case PlannerState::ERROR_STATE:
            return "ERROR_STATE";
        default:
            return "UNKNOWN_STATE";
        }
    }

    enum class SolverState
    {
        SOLVED_WITH_HOMOLOGY_ID, // The mpc formulation was solved with a homology id.
        SOLVED_NO_HOMOLOGY_ID,   // The mpc formulaiton was not solved with a meaningfull homology id.
        SOLVED_FAILD             // The solver failed, a braking command as generated.

    };
}

#endif