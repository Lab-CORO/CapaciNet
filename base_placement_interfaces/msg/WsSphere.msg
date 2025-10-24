# Workspace Sphere - Represents a sphere in the robot's workspace
# with associated reachable poses

std_msgs/Header header
geometry_msgs/Point center    # Center point of the sphere
float32 radius                # Radius of the sphere (ri)
geometry_msgs/Pose[] poses    # Reachable poses from this sphere
