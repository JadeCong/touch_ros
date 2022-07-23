# The touch contains the following states: header, locked, close_gripper, pose, current and velocity.
std_msgs/Header         header
bool                    locked
bool                    close_gripper
geometry_msgs/Pose      pose           # m & rad
geometry_msgs/Vector3   velocity       # m/s
geometry_msgs/Vector3   current        # A
