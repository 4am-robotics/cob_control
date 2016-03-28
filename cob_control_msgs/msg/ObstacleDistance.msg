Header header

## Collision pair consists of link of interest and obstacle
# Registration name of the link of interest
string link_of_interest
# Registration name of the obstacle
string obstacle_id

## distance between the nearest points on obstacle and link of interest
float64 distance

## Vector pointing to the origin of the link
geometry_msgs/Vector3 frame_vector

## Vector pointing to the nearest point on the link collision geometry (e.g. mesh)
geometry_msgs/Vector3 nearest_point_frame_vector

## Vector pointing to the nearest point on the obstacle collision geometry (e.g. mesh)
geometry_msgs/Vector3 nearest_point_obstacle_vector
