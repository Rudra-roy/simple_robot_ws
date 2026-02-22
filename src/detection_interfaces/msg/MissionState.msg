# Mission state information

# Current mission state
string state  # "IDLE", "WAYPOINT_SENT", "NAVIGATING", "GOAL_REACHED", "TASK_EXECUTING", "TASK_COMPLETE", "MISSION_COMPLETE", "ABORTED"

# Current waypoint index (0-based)
int32 current_waypoint_index

# Total number of waypoints in mission
int32 total_waypoints

# Current task being executed (empty if navigation only)
string current_task  # "", "aruco", "mallet", "bottle"

# Mission progress percentage (0.0 to 100.0)
float32 progress
