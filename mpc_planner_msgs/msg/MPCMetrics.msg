Header header
string robot_name

# Solver metrics
float64 solve_time_ms
float64 success_rate           # 0..100 or 0..1 depending on your choice
int32 iterations
int32 exit_code
float64 objective_value
# float64[] objective_values_all_planners 

# Topology metrics  
int32 current_topology_id
int32 previous_topology_id
bool topology_switch
bool used_guidance
int32 selected_planner_index
int32 num_of_guidance_found

# State
string current_state
string previous_state
float64[] current_position # x_pos, y_pos
float64   current_linear_x
float64   current_angular_vel
string[]  planner_names
float64[] planner_objective_values


# Communication metrics
string last_communication_trigger
int32 messages_sent_total
int32 messages_saved_total
float64 communication_savings_percent

# Distribution arrays
int32[]  topology_selection_counts          # e.g. [35, 25, 35, 5]
string[] topology_labels                   # e.g. ["T0", "T1", "T2", "T++"]

int32[]  communication_trigger_counts       # e.g. [45, 25, 15, 10, 5]
string[] communication_trigger_labels      # e.g. ["TIME", "TOPOLOGY", ...]
