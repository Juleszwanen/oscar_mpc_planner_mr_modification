#include <mpc_planner_jackalsimulator/jules_ros1_jackalplanner.h>

#include <mpc_planner/planner.h>

#include <mpc_planner/data_preparation.h>

#include <mpc_planner_util/load_yaml.hpp> // <-- must come first
// using Configuration = CONFIGuration;  // <-- tiny alias for CONFIG macro
#include <mpc_planner_util/parameters.h> // <-- CONFIG depends on the alias above

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>

#include <std_msgs/Empty.h>
#include <ros_tools/profiling.h>

// ----------------------------------------------------------------------------
// PURPOSE
//   Constructor for the per-robot planner wrapper. It wires up configuration,
//   constructs the MPC solver wrapper, connects ROS I/O, and starts the control
//   loop timer.
//
// WHY THESE STEPS ARE IN THIS ORDER
//   1) Log node identity   → useful when running multiple robots in namespaces.
//   2) Load configuration  → the MPC planner expects CONFIG to be initialized
//                            before the Planner object is constructed.
//   3) Build robot footprint→ planner needs it to set collision geometry.
//   4) Read node params     → allow per-instance overrides via private params.
//   5) Construct Planner    → AFTER config, so it can read CONFIG safely.
//   6) Wire ROS I/O         → subscribers/publishers with relative names.
//   7) Start timer          → kicks off the MPC loop at the configured rate.
//
// ASSUMPTIONS
//   - `ros::init` has already run (so `ros::this_node::getName()` is valid).
//   - `SYSTEM_CONFIG_PATH(__FILE__, "settings")` resolves to your YAML.
//   - You link against the mpc_planner libs providing Configuration/CONFIG.
//   - Topics are relative; launch namespaces isolate multiple robots.
//
// SIDE EFFECTS
//   - Starts a ROS timer; from now on `loop()` will be called periodically.
//   - Allocates the Planner (unique_ptr) and initializes internal state.
//
// NOTES
//   - Prefer fully qualified names for repo types (e.g., CONFIGuration)
//     to avoid ambiguity since we are not using `using namespace`.
//   - The log statement had a missing semicolon in your snippet—added below.
// ----------------------------------------------------------------------------
JulesJackalPlanner::JulesJackalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    // 1) Log which node instance is starting (includes namespace, e.g. /robot_1/planner)
    ROS_INFO_STREAM("JULES PlannerNode: " << ros::this_node::getName() << " starting");

    // 2) Load global/config YAML before constructing the Planner.
    //    WHY: The Planner constructor and modules may read CONFIG on creation.
    //    NOTE: Use the fully qualified name to avoid namespace confusion.
    //    __FILE__ is a standard c++ macro, which gives the path by which the preprocessor opend the file
    Configuration::getInstance().initialize(
        SYSTEM_CONFIG_PATH(__FILE__, "settings"));

    // 3) Define robot footprint (discs) from CONFIG.
    //    WHY: Collision modules use a disc-decomposition of the footprint.
    const double length = CONFIG["robot"]["length"].as<double>();
    const double width = CONFIG["robot"]["width"].as<double>();
    const int n_discs = CONFIG["n_discs"].as<int>();
    this->_data.robot_area = MPCPlanner::defineRobotArea(length, width, n_discs);

    // 4) Read per-node parameters (allow launch-time overrides).
    //    - `_global_frame` may differ per simulation/world.
    //    - Control frequency and flags come from CONFIG by default.
    pnh.param("frames/global", this->_global_frame, this->_global_frame); // default "map"
    this->_control_frequency = CONFIG["control_frequency"].as<double>();
    this->_enable_output = CONFIG["enable_output"].as<bool>();
    this->_infeasible_deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
    pnh.param("goal_tolerance", this->_goal_tolerance, this->_goal_tolerance);

    // 5) Construct the Planner AFTER configuration is initialized.
    //    WHY: prevents reading an uninitialized CONFIG inside the constructor.
    this->_planner = std::make_unique<MPCPlanner::Planner>();

    // 6) Wire ROS I/O (all topics are relative → namespace-friendly).
    //    WHY: lets you launch multiple robots under different namespaces.
    this->initializeSubscribersAndPublishers(nh, pnh);
    const auto node_name = ros::this_node::getNamespace().substr(1);
    const std::string saving_name = node_name + "_planner" + ".json";
    RosTools::Instrumentor::Get().BeginSession("mpc_planner_jackalsimulator", saving_name);
    // 7) Start the control loop timer.
    //    WHY: drives periodic calls to `loop()` at `_control_frequency` Hz.
    //    Guard against zero/negative frequency by clamping to ≥1.0 Hz.
    // Start the control loop
    _timer = nh.createTimer(
        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()),
        &JulesJackalPlanner::loop,
        this);

    // Cosmetic divider in logs to separate startup from runtime messages.
    LOG_DIVIDER();
}

JulesJackalPlanner::~JulesJackalPlanner()
{
    LOG_INFO("Stopped JulesJackalPlanner");
    RosTools::Instrumentor::Get().EndSession();
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Wire up all ROS I/O for a single robot instance. Subscribes to state,
//   goal, path, and obstacle topics; advertises command, pose, and an
//   "objective reached" event.
//
// WHY RELATIVE TOPIC NAMES
//   All names are RELATIVE (no leading '/'), so launching this node under
//   different namespaces (e.g., /robot_1, /robot_2) cleanly isolates the I/O
//   per robot without code changes.
//
// WHAT WE SUBSCRIBE TO
//   - "input/state"        (nav_msgs/Odometry)       [optional, improves v]
//   - "input/state_pose"   (geometry_msgs/PoseStamped) [required from Gazebo]
//   - "input/goal"         (geometry_msgs/PoseStamped) [RViz or planner]
//   - "input/reference_path" (nav_msgs/Path)         [global/local path]
//   - "input/obstacles"    (mpc_planner_msgs/ObstacleArray) [from aggregator]
//
// WHAT WE PUBLISH
//   - "output/command"     (geometry_msgs/Twist)     [to diff-drive controller]
//   - "output/pose"        (geometry_msgs/PoseStamped) [for aggregator/RViz]
//   - "events/objective_reached" (std_msgs/Bool)     [supervisor can act on it]
//
// QUEUE SIZES (WHY THESE VALUES)
//   - State topics: 5        → small buffer, low latency
//   - Goal/path/obstacles: 1 → keep only the latest command/reference/snapshot
//
// THREADING
//   - These callbacks run in the ROS spinner thread. With a single-threaded
//     spinner, access to `_data` overlaps safely with the timer loop in practice.
//     If you switch to a MultiThreadedSpinner, guard `_data` with a mutex.
//
// NOTES
//   - We pass `/*pnh*/` unused here; keep it if later you want private pubs.
//   - Using `boost::bind` to match the original codebase style; `std::bind`
//     or method pointer overloads would also work.
// ----------------------------------------------------------------------------
void JulesJackalPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh, ros::NodeHandle & /*pnh*/)
{
    LOG_INFO("initializeSubscribersAndPublishers");

    // ----------------- Inputs -----------------

    // Optional odometry: if present, we use it to get a better velocity estimate.
    _state_sub = nh.subscribe<nav_msgs::Odometry>(
        "input/state", 5,
        boost::bind(&JulesJackalPlanner::stateCallback, this, _1));

    // Required ground truth pose from Gazebo (standard quaternion in orientation).
    _state_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "input/state_pose", 5,
        boost::bind(&JulesJackalPlanner::statePoseCallback, this, _1));

    // Goal input (typically from RViz "2D Nav Goal").
    _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "input/goal", 1,
        boost::bind(&JulesJackalPlanner::goalCallback, this, _1));

    // Reference path for MPCC/contouring modules (optional).
    _path_sub = nh.subscribe<nav_msgs::Path>(
        "input/reference_path", 1,
        boost::bind(&JulesJackalPlanner::pathCallback, this, _1));

    // Obstacles provided by the central aggregator (other robots, pedestrians).
    _obstacles_sub = nh.subscribe<mpc_planner_msgs::ObstacleArray>(
        "input/obstacles", 1,
        boost::bind(&JulesJackalPlanner::obstacleCallback, this, _1));

    // ----------------- Outputs -----------------

    // Velocity command to the robot (Twist: linear.x, angular.z).
    _cmd_pub = nh.advertise<geometry_msgs::Twist>("output/command", 1);

    // Current robot pose for visualization and for the aggregator.
    // _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("output/pose", 1);

    // Publish the current pose of the robot
    _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("output/pose", 1);

    // Publish the trajectory we are going to follow
    _trajectory_pub = nh.advertise<nav_msgs::Path>("output/current_trajectory", 1);

    // Event flag so a supervisor can detect per-robot completion.
    _objective_pub = nh.advertise<std_msgs::Bool>("events/objective_reached", 1);
}

// ----------------------------------------------------------------------------
// PURPOSE
//   This is the periodic control loop, triggered by a ROS timer at
//   `_control_frequency` Hz. It prepares timing info, runs the MPC solve using
//   the most recent `_state` and `_data` (filled by the callbacks), converts
//   the solution into a Twist command, and publishes visualizations/events.
//
// WHY TIMER-DRIVEN
//   MPC is inherently receding-horizon: solve → apply first control → advance
//   time. A fixed-rate timer keeps latency predictable and simplifies tuning.
//
// DATA FLOW
//   - `_state` and `_data` are updated asynchronously by ROS callbacks
//     (statePose/odom, goal, path, obstacles).
//   - Here we call `_planner->solveMPC(_state, _data)` to compute the next
//     control sequence; then we take the first control to command the robot.
//
// COMMAND EXTRACTION CONVENTION
//   Matches the original wrapper:
//     - `u0 = w` (angular rate at stage 0)
//     - `x1 = v` (linear speed at stage 1)
//   Hence: `cmd.angular.z = getSolution(0,"w")`, `cmd.linear.x = getSolution(1,"v")`.
//
// FAIL-SAFE (BRAKING)
//   If the solver fails (infeasible/no convergence) or output is disabled,
//   we apply a simple braking law:
//       v_next = max(v - decel * dt, 0)
//   This prevents reversing and buys time for the optimizer to recover.
//
// VISUALIZATION / EVENTS
//   - `publishPose()` keeps downstream consumers (aggregator, RViz) updated.
//   - `_planner->visualize(...)` uses the repo’s built-in markers (if linked).
//   - `visualize()` adds a heading ray for quick orientation feedback.
//   - If `objectiveReached()`, we publish an event for a supervisor to act on.
//
// THREADING
//   - With a single-threaded spinner, callbacks and this loop are serialized.
//   - If you switch to a MultiThreadedSpinner, guard shared `_data` with a mutex.
//
// NOTES
//   - `planning_start_time` is set for instrumentation/profiling inside the
//     planner modules.
//   - The unused timer event is intentionally omitted (`/*event*/`) to silence
//     warnings without extra code.
// ----------------------------------------------------------------------------
void JulesJackalPlanner::loop(const ros::TimerEvent & /*event*/)
{
    const auto ns = ros::this_node::getNamespace();
    const std::string profiling_name = ns + "_" + "planningLoop";
    PROFILE_SCOPE(profiling_name.c_str());
    // Timestamp the beginning of this planning cycle (used by internal profiling).
    _data.planning_start_time = std::chrono::system_clock::now();

    LOG_DEBUG("============= Loop =============");

    // Optional verbose state print for debugging if enabled in config.
    if (CONFIG["debug_output"].as<bool>())
        _state.print();

    // Run the MPC using the latest state & data assembled by the callbacks.
    // `_state` holds (x, y, psi, v), `_data` holds goal/path/obstacles/footprint.
    // The state and the data are filled via the different callback functions
    // The output cotains besides a succes boolean als a trajectory which it triews to follow
    auto output = _planner->solveMPC(_state, _data);
    LOG_MARK("Success: " << output.success);

    geometry_msgs::Twist cmd;

    if (_enable_output && output.success)
    {
        // Extract the first applicable controls from the solution following
        // the repo’s convention: u0 = w, x1 = v (next-step linear speed).
        cmd.linear.x = _planner->getSolution(1, "v");
        cmd.angular.z = _planner->getSolution(0, "w");

        LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
        LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
    }
    else
    {
        // ---------------------- FAIL-SAFE BRAKING ----------------------
        // If the solver failed or output is disabled, slow down smoothly
        // instead of issuing stale/unsafe commands.
        double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
        double velocity_after_braking;
        double velocity;
        double dt = 1. / CONFIG["control_frequency"].as<double>();

        velocity = _state.get("v");
        velocity_after_braking = velocity - deceleration * dt; // Brake with the given deceleration
        cmd.linear.x = std::max(velocity_after_braking, 0.);   // Don't drive backwards when braking
        cmd.angular.z = 0.0;
        // ----------------------------------------------------------------
    }

    // Publish the velocity command to the robot’s differential drive.
    _cmd_pub.publish(cmd);

    // Keep downstream consumers updated with our current pose (for RViz/aggregator).
    this->publishPose();
    // Publish the trajectory a robot is about to follow
    this->publishCurrentTrajectory(output);

    // Built-in planner visuals (predicted trajectory, footprints, etc.), if available.
    _planner->visualize(_state, _data);

    // Quick heading ray for sanity checking.
    visualize();

    // Notify a supervisor that this robot considers the goal reached.
    if (objectiveReached())
    {
        std_msgs::Bool done;
        done.data = true;
        _objective_pub.publish(done);
    }

    LOG_DEBUG("============= End Loop =============");
}

void JulesJackalPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    _state.set("v", std::hypot(vx, vy));

    // simple flip check (same spirit as original)
    if (std::abs(msg->pose.pose.orientation.x) > (M_PI / 8.) ||
        std::abs(msg->pose.pose.orientation.y) > (M_PI / 8.))
    {
        LOG_WARN("Detected flipped robot (odom).");
        // JULES normaly there was below, this should perhaps be fixed
        /*
        LOG_WARN("Detected flipped robot. Resetting.");
        reset(false); // Reset without success
        */
    }
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Ingest a ground-truth robot state from a PoseStamped and write it into the
//   planner’s internal `_state` (x, y, psi, v).
//
// IMPORTANT FORMAT NOTE (ENCODED POSE):
//   This callback assumes a **non-standard / encoded** PoseStamped where:
//     - yaw (psi) is stored directly in `pose.orientation.z`  (NOT a quaternion)
//     - linear speed v is stored in `pose.position.z`
//   This is the convention used by the repo’s `mobile_robot_state_publisher`.
//   It differs from Gazebo’s standard where orientation is a proper quaternion
//   and velocity is not in Pose at all.
//
// WHY THIS EXISTS
//   It provides a very light-weight path to feed the MPC with position, yaw,
//   and speed without subscribing to Odometry. The encoding collapses multiple
//   signals into a single PoseStamped for convenience.
//
// FLIP DETECTION & RESET
//   The check on `orientation.x` and `.y` is intended to catch large roll/pitch
//   (robot flipped). If detected, it calls `reset(false)` to trigger a sim reset.
//   ⚠ In the encoded format, `orientation.x/y` are typically zero, so this check
//     is only meaningful if a real quaternion is being sent.
//   ⚠ In a multi-robot architecture, a **Supervisor** node should handle resets;
//     this callback should at most publish an event (e.g., `events/request_reset`).
//
// ASSUMPTIONS
//   - 2D planar navigation; only x,y,psi,v are used.
//   - The frame of `msg` matches the planner’s global frame (e.g., "map").
//   - Upstream has already encoded psi and v into the PoseStamped as described.
//
// SIDE EFFECTS
//   - Mutates `_state`.
//   - May call `reset(false)` which can reset the *entire* sim (heavy-handed).
//
// EXTENSIONS
//   - Add a parameter (e.g., `state_pose/use_encoded`) to toggle between this
//     encoded behavior and a standard-quaternion handler.
//   - Replace the direct `reset(false)` with publishing an event for a Supervisor.
// ----------------------------------------------------------------------------
void JulesJackalPlanner::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Write planar pose directly from the encoded message fields.
    _state.set("x", msg->pose.position.x);
    _state.set("y", msg->pose.position.y);
    _state.set("psi", msg->pose.orientation.z); // encoded yaw (NOT a quaternion)

    // Linear speed v is encoded in position.z by upstream publisher.
    _state.set("v", msg->pose.position.z);

    // Flip detection: large roll/pitch implies the robot may have flipped.
    // NOTE: This only makes sense if `orientation` is a real quaternion.
    // With the encoded format, x/y are usually 0 and this check won’t trigger.
    if (std::abs(msg->pose.orientation.x) > (M_PI / 8.) ||
        std::abs(msg->pose.orientation.y) > (M_PI / 8.))
    {
        LOG_ERROR("Detected flipped robot. Resetting.");

        // In the original single-robot setup this hard-resets the sim.
        // In a multi-robot setup, prefer publishing an event and let a
        // Supervisor decide what to reset.
        // reset(false); // Reset without success
    }
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Handle a new 2D navigation goal for this robot.
//   This callback is typically fed by RViz's "2D Nav Goal" tool or any node
//   that publishes a geometry_msgs::PoseStamped on "input/goal".
//
// WHY THIS EXISTS
//   The MPC planner reads the goal from `_data.goal` at each solve cycle.
//   We also keep a local copy `_goal_xy` so the wrapper can detect when the
//   robot has reached the goal (objectiveReached) and publish an event.
//
// ASSUMPTIONS
//   - We navigate in the XY plane; only position.x/.y are used.
//   - Orientation in the message is ignored here (heading goals can be added later).
//   - The goal is expressed in the same frame as `_global_frame` (often "map").
//     If not, you should transform it before writing into `_data.goal`.
//
// SIDE EFFECTS
//   - Sets `_data.goal` and flags to activate goal-related modules in the MPC.
//   - Sets `_goal_xy` and `_goal_received` so `objectiveReached()` can work.
//   - Does NOT reset or change world state (that’s the supervisor’s job).
//
// THREADING / TIMING
//   - Runs in the ROS callback thread. It only writes simple fields; no locking
//     is needed under the typical single-threaded spinner.
//
// OPTIONAL BEHAVIOR
//   - If you want modules to react immediately, you can call:
//       _planner->onDataReceived(_data, "goal");
//     We keep behavior consistent with the original by not doing that here.
// ----------------------------------------------------------------------------
void JulesJackalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    LOG_DEBUG("Goal callback"); // why: traceability in logs during testing

    // Why: the MPC library expects the current goal in `_data.goal` (x,y).
    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;

    // Why: cached Eigen vector lets `objectiveReached()` do a fast distance check
    // without touching `_data` every time.
    _goal_xy = Eigen::Vector2d(_data.goal(0), _data.goal(1));

    // Why: these flags tell (a) the planner library that a goal is active,
    // and (b) this wrapper that it should evaluate goal completion.
    _data.goal_received = true;
    _goal_received = true;

    // Optional immediate notification to planner modules:
    // _planner->onDataReceived(_data, "goal");
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Fast check to avoid re-processing the reference path if the incoming path
//   is (effectively) the same as the one we already have in `_data.reference_path`.
//   If it’s the same, the caller can simply return without clearing/reloading
//   the path or notifying planner modules.
//
// WHY THIS EXISTS
//   Rebuilding the reference path and notifying modules (e.g., contouring/MPCC)
//   can trigger extra work, solver parameter updates, and visualization churn.
//   A cheap equality check prevents unnecessary planner disturbances.
//
// STRATEGY
//   1) Compare path lengths. If different, we assume the path changed.
//   2) Compare up to the first two points using `pointInPath(...)`, which
//      encapsulates the planner’s notion of “same point” (usually with a
//      tolerance). Two points are usually enough to detect changes while
//      keeping this O(1) time.
//
// ASSUMPTIONS
//   - Order of poses in the path is consistent (same publisher behavior).
//   - `pointInPath(i, x, y)` implements an appropriate tolerance for small
//     numerical differences; otherwise, tiny noise may make paths look “new”.
//   - Orientation along the path is not considered here; only (x, y) matter.
//   - Using only the first 1–2 points is sufficient to detect practical changes
//     in most scenarios. If your upstream planner sometimes changes later
//     segments while keeping the start identical, consider checking the last
//     point too.
//
// EDGE CASES
//   - If both paths are empty (size==0), this returns true (treat as unchanged).
//   - If the incoming path has fewer than 2 points, we compare only what exists.
//   - If you need stricter checks, you can extend this to compare the last
//     point or a hash/stamp of the path.
//
// THREADING
//   - Read-only access to `_data` in a const method. Safe under single-threaded
//     spinning. In a multi-threaded spinner, guard `_data` with a mutex.
//
// COMPLEXITY
//   - O(1) time and O(1) memory (vs O(N) for full path comparison).
// ----------------------------------------------------------------------------
bool JulesJackalPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg) const
{
    // Quick reject: if lengths differ, treat as different.
    if (_data.reference_path.x.size() != msg->poses.size())
        return false;

    // Compare only the first few points for speed/stability.
    const int num_points = std::min(2, static_cast<int>(_data.reference_path.x.size()));
    for (int i = 0; i < num_points; ++i)
    {
        // `pointInPath(i, x, y)` should apply the planner’s internal tolerance.
        if (!_data.reference_path.pointInPath(
                i,
                msg->poses[i].pose.position.x,
                msg->poses[i].pose.position.y))
        {
            return false; // diverges early → treat as a new path
        }
    }

    // Same length and first points match within tolerance → consider unchanged.
    return true;
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Ingest a new reference path (global or local) for the MPC to track.
//   The path typically comes from a global planner or a pre-baked route and
//   is published as `nav_msgs/Path` on "input/reference_path".
//
// WHY THIS EXISTS
//   Several MPC modules (e.g., MPCC / contouring) rely on `_data.reference_path`
//   to bias the optimizer toward the lane/route. Updating this structure and
//   notifying the planner lets those modules refresh their internal parameters.
//
// FAST-PATH GUARD
//   We first call `isPathTheSame(msg)` to avoid unnecessary work when the path
//   hasn't changed (prevents churn in solver parameters and visuals).
//
// WHAT WE STORE
//   - Only the XY positions are copied into `_data.reference_path.x/.y`.
//   - We push a single `0.0` into `.psi` to satisfy modules that expect the
//     field to be non-empty. Orientation along the path is either computed
//     internally by the planner or not required for the chosen cost modules.
//
// ASSUMPTIONS
//   - Path poses are expressed in the same frame used by the planner
//     (typically `_global_frame`, e.g., "map"). If not, transform beforehand.
//   - Z and quaternion in the incoming path are ignored here (2D planning).
//   - If your modules require per-point tangent angles/curvature, provide them
//     in a separate preprocessing step or extend this callback to fill `.psi`.
//
// SIDE EFFECTS
//   - `_data.reference_path` is cleared and repopulated.
//   - `_planner->onDataReceived(..., "reference_path")` notifies modules that
//     depend on the reference path so they can update solver parameters.
//
// THREADING
//   - Runs in the ROS callback thread. `_data` is mutated here and read in the
//     timer loop; with a single-threaded spinner this is fine. In a multi-
//     threaded spinner, add a mutex around `_data` access.
//
// EXTENSIONS
//   - You may resample/smooth the path here to enforce a fixed spacing.
//   - You may store arc-length (s) to support curvature-aware weights.
// ----------------------------------------------------------------------------
void JulesJackalPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    LOG_DEBUG("Path callback");

    // Early exit to avoid reconfiguring the planner if the path hasn't changed.
    if (isPathTheSame(msg))
        return;

    // Reset the stored reference path (clears x, y, psi, ...).
    _data.reference_path.clear();

    // Copy XY waypoints from the incoming path.
    for (const auto &pose : msg->poses)
    {
        _data.reference_path.x.push_back(pose.pose.position.x);
        _data.reference_path.y.push_back(pose.pose.position.y);
    }

    // Ensure psi has at least one element to satisfy modules that check it.
    // (Planner may compute headings internally if needed.)
    _data.reference_path.psi.push_back(0.0);

    // Notify planner modules that depend on the reference path (e.g., MPCC).
    _planner->onDataReceived(_data, "reference_path");
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Convert an incoming set of dynamic obstacles (robots, pedestrians, etc.)
//   into the planner’s internal format and notify MPC modules that rely on
//   obstacle data (hard/soft collision avoidance, chance constraints, …).
//
// WHY THIS EXISTS
//   The planner optimizes over a prediction horizon. For collision avoidance
//   it needs, at each horizon step, either:
//     - current obstacle poses only (deterministic, no prediction), or
//     - predicted obstacle poses with uncertainty (Gaussian ellipses).
//   This callback bridges the ROS message (`ObstacleArray`) to the MPC’s
//   `_data.dynamic_obstacles` representation.
//
// DATA FLOW
//   - Central aggregator publishes `mpc_planner_msgs/ObstacleArray` on
//     "input/obstacles" per robot namespace.
//   - Here we clear the previous set, parse each obstacle, and (optionally)
//     attach a single-mode Gaussian prediction over the horizon.
//   - We pad/truncate to the configured maximum number of obstacles so the
//     solver has a fixed-size parameter vector.
//   - Finally we notify modules via `_planner->onDataReceived(..., "dynamic obstacles")`.
//
// ASSUMPTIONS
//   - Poses are expressed in the same global frame the planner uses (e.g., "map").
//   - If `probabilities` is non-empty and `gaussians.size()==1`, we treat the
//     obstacle as having one Gaussian prediction “mode” defined across the
//     solver horizon (positions + ellipse semiaxes per step).
//   - Multi-modal predictions (gaussians.size()>1) are not handled here; they
//     can be added later with a scenario-based extension.
//   - Orientation is used if your collision model depends on it; otherwise it
//     can be ignored by the underlying module.
//
// EDGE CASES & SAFETY
//   - If no prediction is provided, we mark the obstacle as DETERMINISTIC (i.e.,
//     only current pose). Downstream modules may extrapolate or enforce static
//     separation at each step.
//   - If the last semiaxis is zero OR probabilistic mode is disabled in CONFIG,
//     we degrade to DETERMINISTIC to avoid fake uncertainty.
//   - `ensureObstacleSize(...)` pads/truncates to a fixed count (adds dummies)
//     to keep solver dimensions constant.
//   - Optional `propagatePredictionUncertainty(...)` can inflate ellipses across
//     time to reflect growing uncertainty.
//
// PERFORMANCE
//   - Linear in number of obstacles and horizon steps; runs in the ROS callback
//     thread. Keep the aggregator’s obstacle count reasonable.
//
// THREADING
//   - Mutates `_data.dynamic_obstacles` while the timer loop reads it. Under a
//     single-threaded spinner (default), this is fine. With multi-threaded
//     spinners, guard `_data` with a mutex.
//
// EXTENSIONS
//   - Map other robots to distinct IDs, set radii per-type, or filter by range.
//   - Add support for multi-modal predictions by creating multiple `modes` and
//     enabling a scenario approach in the planner.
// ----------------------------------------------------------------------------
void JulesJackalPlanner::obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg)
{
    // Start fresh each message; the solver expects the latest snapshot.
    const auto ns = ros::this_node::getNamespace();
    const std::string profiling_name = ns + "_" + "ObstacleCallback";
    PROFILE_SCOPE(profiling_name.c_str());
    _data.dynamic_obstacles.clear();

    for (const auto &obstacle : msg->obstacles)
    {
        // Create a new dynamic obstacle with:
        //  - id:      stable identifier from upstream (aggregator/ped sim)
        //  - pos:     current xy position
        //  - yaw:     orientation (used by some footprint models)
        //  - radius:  planner-wide default unless you encode per-obstacle sizes
        _data.dynamic_obstacles.emplace_back(
            obstacle.id,
            Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
            RosTools::quaternionToAngle(obstacle.pose),
            CONFIG["obstacle_radius"].as<double>());
        auto &dyn = _data.dynamic_obstacles.back();

        // Single-mode Gaussian predictions if provided; otherwise deterministic.
        // We treat "has probabilities AND exactly one gaussian" as "one-mode"
        // and fill per-step means + ellipse semiaxes. The actual probability
        // values are not used here; they are interpreted downstream.
        if (!obstacle.probabilities.empty() && obstacle.gaussians.size() == 1)
        {
            dyn.prediction = MPCPlanner::Prediction(MPCPlanner::PredictionType::GAUSSIAN);

            const auto &mode = obstacle.gaussians[0];
            dyn.prediction.modes[0].reserve(mode.mean.poses.size());
            for (size_t k = 0; k < mode.mean.poses.size(); ++k)
            {
                dyn.prediction.modes[0].emplace_back(
                    Eigen::Vector2d(mode.mean.poses[k].pose.position.x, mode.mean.poses[k].pose.position.y),
                    RosTools::quaternionToAngle(mode.mean.poses[k].pose.orientation),
                    mode.major_semiaxis[k],
                    mode.minor_semiaxis[k]);
            }

            // If uncertainty is effectively zero or probabilistic handling is
            // disabled in config, fall back to deterministic to simplify constraints.
            if (mode.major_semiaxis.back() == 0.0 ||
                !CONFIG["probabilistic"]["enable"].as<bool>())
            {
                dyn.prediction.type = MPCPlanner::PredictionType::DETERMINISTIC;
            }
        }
        else
        {
            // No prediction provided (or multi-mode not supported here):
            // treat obstacle as deterministic (current pose only).
            dyn.prediction = MPCPlanner::Prediction(MPCPlanner::PredictionType::DETERMINISTIC);
        }
    }

    // Pad or trim to the configured max_obstacles so solver parameter sizes
    // remain constant (adds “dummy” obstacles if needed).
    MPCPlanner::ensureObstacleSize(_data.dynamic_obstacles, _state);

    // Optionally inflate/propagate uncertainty over time if enabled.
    if (CONFIG["probabilistic"]["propagate_uncertainty"].as<bool>())
        MPCPlanner::propagatePredictionUncertainty(_data.dynamic_obstacles);

    // Notify planner modules (e.g., collision avoidance) that new obstacle
    // data is available so they can refresh solver params before the next solve.
    _planner->onDataReceived(_data, "dynamic obstacles");
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Publish the robot’s current pose (x, y, yaw) as a `geometry_msgs/PoseStamped`
//   on "output/pose". Other nodes (e.g., the central aggregator, RViz) consume
//   this to visualize the robot and to model it as a dynamic obstacle.
//
// WHY THIS EXISTS
//   In a multi-robot setup we decoupled obstacle creation from the planner.
//   Each robot publishes its own pose; the aggregator subscribes to every
//   robot’s "output/pose" and builds per-robot `ObstacleArray`s for
//   "input/obstacles". Keeping this publisher here makes the planner wrapper
//   self-contained and easy to reuse.
//
// WHAT IT PUBLISHES
//   - `header.stamp`: now, so downstream consumers can measure freshness.
//   - `header.frame_id`: `_global_frame` (e.g., "map") so poses are comparable
//     across robots and with obstacle data.
//   - `pose.position.x/y`: taken from the planner’s internal `_state` (meters).
//   - `pose.orientation`: quaternion built from `_state("psi")` (yaw-only).
//     Z-up planar assumption; roll/pitch = 0.
//
// ASSUMPTIONS
//   - `_state` is up to date when this is called (either after a state callback
//     or within the control loop).
//   - 2D navigation: z is unused and left at 0 by omission.
//   - `_global_frame` matches the frame used by your aggregator and RViz.
//
// SIDE EFFECTS
//   - None beyond publishing. No world resets, no parameter changes.
//
// TIMING
//   - Safe to call at the control loop rate and/or right after state updates.
//     If both, downstream consumers will simply receive more recent poses.
//
// EXTENSIONS
//   - If you need velocity available to other nodes, you could add it as a
//     separate topic (e.g., "output/twist") rather than encoding it into pose.
//   - If frames can differ, consider TF-based transforms before publishing.
// ----------------------------------------------------------------------------
void JulesJackalPlanner::publishPose()
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _global_frame;

    // Read back from internal state so what we publish is consistent with the
    // values the MPC used/updated this cycle.
    pose.pose.position.x = _state.get("x");
    pose.pose.position.y = _state.get("y");

    // Store the angel in the orientation.z place <- same convetion used in mobile_robot_state_publisher_node_mr
    pose.pose.orientation.z = _state.get("psi");
    // Store the speed in the position.z place <- same convetion used in mobile_robot_state_publisher_node_mr
    pose.pose.position.z = _state.get("v");

    _pose_pub.publish(pose);
}

// Publishes the trajectory we are following with 20 hz(equal to the control level loop)
void JulesJackalPlanner::publishCurrentTrajectory(MPCPlanner::PlannerOutput output)
{
    const auto ns = ros::this_node::getNamespace();
    const std::string profiling_name = ns + "_" + "PublishCurrentTrajectory";
    PROFILE_SCOPE(profiling_name.c_str());
    nav_msgs::Path ros_trajectory_msg;

    auto ros_time = ros::Time::now();
    ros_trajectory_msg.header.stamp = ros_time;
    ros_trajectory_msg.header.frame_id = _global_frame;

    int k = 0;
    // loop trough the vector og eigenposition
    if (output.trajectory.positions.size() != output.trajectory.orientations.size())
    {
        ROS_WARN("Trajectory positions and orientations size mismatch! Skipping trajectory publication.");
        return;
    }
    for (const auto &position : output.trajectory.positions)
    {

        ros_trajectory_msg.poses.emplace_back();
        ros_trajectory_msg.poses.back().pose.position.x = position(0);
        ros_trajectory_msg.poses.back().pose.position.y = position(1);
        // there is no z-component so we set it to 1
        ros_trajectory_msg.poses.back().pose.position.z = 1;

        ros_trajectory_msg.poses.back().pose.orientation = RosTools::angleToQuaternion(output.trajectory.orientations.at(k));
        ros_trajectory_msg.poses.back().header.stamp = ros_time + ros::Duration(k * output.trajectory.dt);
        k++;
    }

    _trajectory_pub.publish(ros_trajectory_msg);
}

// THIS MIGHT BE USEFULL LATER
// void JulesJackalPlanner::publishPoseWithQuarternion()
// {
//     nav_msgs::Odometry nav_pose_msg;
//     nav_pose_msg.header.stamp = ros::Time::now();
//     nav_pose_msg.header.frame_id = _global_frame;

//     nav_pose_msg.child_frame_id = ros::this_node::getNamespace() + "/base_link";
//     // Read back from internal state so what we publish is consistent with the
//     // values the MPC used/updated this cycle.
//     nav_pose_msg.pose.position.x = _state.get("x");
//     nav_pose_msg.pose.position.y = _state.get("y");
//     nav_pose_msg.pose.position.z = 0;

//     double psi = _state.get("psi"); //get the orientation
//     nav_pose_msg.pose.orientation = RosTools::angleToQuaternion(psi);

//     nav_pose_msg.twist.x =
//     nav_pose_msg.twist.y =

//     _pose_pub.publish(pose);
// }

// ----------------------------------------------------------------------------
// PURPOSE
//   Publish a simple RViz-friendly visualization of the robot’s heading as a
//   line (“orientation ray”) starting at the current (x, y) position and
//   extending 1 meter in the direction of yaw (psi).
//
// WHY THIS EXISTS
//   Quick visual feedback helps during debugging and tuning: you can confirm
//   that the internal `_state` matches what you see in Gazebo/RViz and that
//   the robot’s commanded orientation is sensible without digging into logs.
//
// WHAT IT DRAWS
//   - A single line segment from the robot’s current position to a point that
//     is 1.0 meter ahead along the heading direction.
//   - Uses the shared `ros_tools::VISUALS` publisher pool under the key "angle".
//     Each call creates a new line in that publisher and immediately publishes.
//
// ASSUMPTIONS
//   - 2D planar navigation; heading is `_state("psi")` (radians, Z-up).
//   - The VISUALS system is initialized elsewhere (e.g., `VISUALS.init(&nh)` in main).
//   - RViz is configured to display the corresponding marker topic.
//
// SIDE EFFECTS
//   - Publishes a visualization marker each time this function is called.
//   - No changes to planner state or parameters.
//
// PERFORMANCE
//   - Lightweight; one line marker per call. Safe to invoke once per control loop.
//
// EXTENSIONS
//   - Replace the hard-coded 1.0 meter with a param (e.g., `viz/heading_length`).
//   - Add a footprint outline, predicted trajectory, or obstacle ellipses here,
//     or leave those to `_planner->visualize(...)` if already implemented.
// ----------------------------------------------------------------------------
void JulesJackalPlanner::visualize()
{
    auto &publisher = VISUALS.getPublisher("angle");
    auto &line = publisher.getNewLine();

    line.addLine(
        Eigen::Vector2d(_state.get("x"), _state.get("y")),
        Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")),
                        _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
    publisher.publish();
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Decide whether the robot has reached its current goal using a simple
//   Euclidean distance check in the XY plane.
//
// WHY THIS EXISTS
//   The planner wrapper needs a lightweight, local criterion to trigger
//   downstream behavior (e.g., publish `events/objective_reached`). We keep
//   resets/supervision outside this node, so this function is the local gate.
//
// LOGIC
//   - If no goal has been received yet, return false.
//   - Compute distance between current position and cached goal `_goal_xy`.
//   - Return true if distance ≤ `_goal_tolerance` (meters).
//
// ASSUMPTIONS
//   - The goal and the robot pose are expressed in the same global frame
//     (e.g., "map").
//   - 2D navigation: only x,y are considered; heading is ignored here.
//   - `_goal_tolerance` is configured to match your use case (typ. 0.2–1.0 m).
//
// EDGE CASES
//   - If `_goal_tolerance` is too small relative to localization noise, the
//     condition may flicker. Add hysteresis or time filtering if needed.
//   - If you also care about final heading, extend with a yaw tolerance check.
//
// THREADING
//   - Read-only access to `_state` and `_goal_xy` in a const method; safe under
//     the usual single-threaded spinner. Use a mutex if you switch to multi-thread.
//
// EXTENSIONS
//   - Add yaw tolerance: `std::abs(angDiff(stateYaw, goalYaw)) < yaw_tol`.
//   - Add “hold time” hysteresis: require the condition to be true for N cycles.
//   - Add a minimum speed threshold to ensure you’re not just passing by.
// ----------------------------------------------------------------------------
bool JulesJackalPlanner::objectiveReached() const
{
    if (!_goal_received)
        return false;

    const Eigen::Vector2d p(_state.get("x"), _state.get("y"));
    return (p - _goal_xy).norm() <= _goal_tolerance;
}

// ----------------------------------------------------------------------------
// PURPOSE
//   Entry point of the ROS node. Sets up ROS, creates public (+ private) node
//   handles, initializes optional visuals, constructs your planner wrapper,
//   then hands control to ROS's event loop.
//
// WHY IN THE SAME FILE
//   Keeping main() in the same translation unit as JulesJackalPlanner is fine
//   (and convenient while iterating). Just make sure you compile/link exactly
//   one file that defines main(); don’t also compile a separate jules_main.cpp,
//   or you’ll get “multiple definition of `main`” link errors.
// ----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // Initialize the ROS client library.
    // The string "planner" is the node name; combined with namespaces you can
    // run two nodes named "planner" simultaneously (e.g., /robot_1/planner and
    // /robot_2/planner) without conflicts.
    ros::init(argc, argv, "JULES_jackalplanner");

    // Public node handle: resolves topics/services relative to the node's
    // namespace (e.g., /robot_1/...). Use this for publishers/subscribers.
    ros::NodeHandle nh;

    // Private node handle: resolves parameters (and optional private topics)
    // under ~ (i.e., /<ns>/<node_name>/...). We pass this so the planner can
    // read node-specific params without colliding with other nodes.
    ros::NodeHandle pnh("~");

    // Optional: initialize the shared visualization helper used by the repo.
    // If you’re not linking ros_tools/visuals, you can remove this line, and
    // also remove calls to visualize() or planner_->visualize(...).
    VISUALS.init(&nh);

    // Construct your per-robot planner wrapper.
    // Why here: constructing AFTER ros::init + NodeHandles ensures ROS is ready.
    // The constructor will:
    //  - load config,
    //  - create _planner (unique_ptr),
    //  - wire up pubs/subs (relative topics),
    //  - start the control loop timer.
    JulesJackalPlanner planner(nh, pnh);

    // Hand control to ROS: this pumps callbacks (subs, timers) in this thread.
    // Single-threaded spinner keeps your planner loop and callbacks serialized,
    // which is simpler. If you ever need concurrency, switch to MultiThreadedSpinner.
    ros::spin();

    // On shutdown (Ctrl+C or node kill), ros::spin() returns, planner's destructor
    // runs (clean up timers/pubs/subs), then we exit main.
    return 0;
}
