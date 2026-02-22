# Detection result message for all detection nodes (aruco, mallet, bottle)

std_msgs/Header header

# Detection type identifier
string detection_type  # "aruco", "mallet", "bottle"

# Detection success flag
bool success

# Target pose relative to robot (when detected)
geometry_msgs/Pose target_pose

# Detection confidence (0.0 to 1.0)
float32 confidence

# Distance to target in meters
float32 distance

# Current state of detection process
string state  # "searching", "tracking", "reached", "failed"
