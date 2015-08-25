Header header

# Collision pair: frame of interest and obstacle
## Registration name of the link of interest
string link_of_interest

## Registration name of the obstacle
string obstacle_id

# distance between the nearest points on obstacle and frame of interest
float64 distance

# Vector pointing to the origin of the link (e.g. arm_right_7_link)
geometry_msgs/Vector3 frame_vector

# Vector pointing to the nearest point on the frame
geometry_msgs/Vector3 nearest_point_frame_vector

# Vector pointing to the nearest point on the obstacle
geometry_msgs/Vector3 nearest_point_obstacle_vector