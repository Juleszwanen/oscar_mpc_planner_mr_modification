# ID for obstacle association
int32 id

# Jules: boolean to check if we already send this specific obstacle trajectory in an earlier iteration, this can be used to identify if we plan with an old trajectory
# bool updated_trajectory_prediction
# Current pose of the obstacle
geometry_msgs/Pose pose

# List of Gaussians and their probabilities
mpc_planner_msgs/Gaussian[] gaussians
float64[] probabilities

