The `cob_collision_velocity_filter` reads the obstacles from a
rolling-window costmap and the velocity commands published by a teleop
device. It then computes relevant obstacles in driving direction and
slows down if it gets closer to them. It then stops the robot at a
specific threshold.

**Note:** The `cob_collision_velocity_filter` only works with
rectangular footprints at the moment.

## Hardware Requirements

To use this package you need either need a real or a simulated
Care-O-Bot (see [cob_bringup](https://index.ros.org/p/cob_bringup) and
[cob_bringup_sim](https://index.ros.org/p/cob_bringup_sim)
respectively).

## ROS API

The
cob_collision_velocity_filter
package provides a configurable node for collision avoidance using
teleoperation devices

### cob_collision_velocity_filter

The `cob_collision_velocity_filter` node takes in
[geometry_msgs/Twist](http://ros.org/doc/api/geometry_msgs/html/msg/Twist.html)
messages from the teleop device as well as
[nav_msgs/GridCells](http://ros.org/doc/api/nav_msgs/html/msg/GridCells.html)
messages specifying the obstacles on the costmap. The safe movement
commands are published as a [topic](Topics), as well as the extracted
relevant obstacles in driving direction. It further periodically tries
to call the `/get_footprint` service of
[cob_footprint_observer](https://index.ros.org/p/cob_footprint_observer)
to check for changed footprints and adjusts the used one accordingly.

#### Subscribed Topics

- `teleop_twist`
  ([geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)):
  unsafe movement command of the teleop device
- `obstacles`
  ([nav_msgs/msg/GridCells](https://docs.ros.org/en/rolling/p/nav_msgs/msg/GridCells.html)):
  obstacles on a rolling-window costmap

#### Published Topics

- `command`
  ([geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)):
  safe movement command
- `relevant_obstacles`
  ([nav_msgs/msg/GridCells](https://docs.ros.org/en/rolling/p/nav_msgs/msg/GridCells.html)):
  obstacles in driving direction

#### Parameters

- `~costmap_parameter_source` (`string`, default:
  `"/local_costmap_node/costmap"`): Specifies which node to search for
  the initial footprint, footprint_padding, etc., e.g.
  /local_costmap_node/costmap.
- `~footprint_update_frequency` (`double`, default: `1.0 [Hz]`):
  Frequency at which footprint is adjusted using the `/get_footprint`
  service from `cob_footprint_observer`.
- `~pot_ctrl_vmax` (`double`, default: `0.6`): Maximum allowed velocity
  for safe movements.
- `~pot_ctrl_vtheta_max` (`double`, default: `0.8`): Maximum allowed
  rotational velocity for safe movements.
- `~pot_ctrl_kv` (`double`, default: `1.0`): Damping term for potential
  field like controller for slow down behaviour.
- `~pot_ctrl_kp` (`double`, default: `2.0`): Stiffness term for
  potential field like controller for slow down behaviour.
- `~pot_ctrl_virt_mass` (`double`, default: `0.8`): Virtual mass term
  for potential field like controller for slow down behaviour.
- `~influence_radius` (`double`, default: `1.5 [m]`): max distance of
  obstacles from robot center that are used for computing relevant
  obstacles.
- `~obstacle_damping_dist` (`double`, default: `5.0 [m]`): Distance in
  driving direction at which potential field like slow down controller
  starts to work.
- `~stop_threshold` (`double`, default: `0.1 [m]`): Distance to
  `relevant_obstacle` at which robot stops to move completely.
- `~use_circumscribed_threshold` (`double`, default: `0.2 [rad/s]`):
  Minimal rotational velocity at which circumscribed_radius should be
  used for collision avoidance while translational velocities are
  present. Has no influence when robot is only moving rotationally
  because then it is the rotational movement is the only critical one
  and thus is taken into account nonetheless.

## Usage/Examples

For starting the `cob_collision_velocity_filter` use


```
roslaunch cob_bringup base_collision_observer.launch
```

 This launches the required rolling window
[costmap_2d](https://index.ros.org/p/costmap_2d) as well as the optional
[cob_footprint_observer](https://index.ros.org/p/cob_footprint_observer).
If you don't want to use the
[cob_footprint_observer](https://index.ros.org/p/cob_footprint_observer)
simply comment or delete the respective lines in the launch file. For
including the `collision_velocity_filter` in your overall launch file
use 

```
<include file=$(find cob_bringup/tools/base_collision_observer.launch />
```

 A sample parameter file for the
cob_collision_velocity_filter
can look like this: 

```
#where to look for costmap parameters
costmap_parameter_source: "/local_costmap_node/costmap" #default "/local_costmap_node/costmap"

#frequency at which the get_footprint service should be called
footprint_update_frequency: 0.5

#Parameters specifying slow down behaviour
pot_ctrl_vmax: 0.6 #default: 0.6
pot_ctrl_vtheta_max: 0.6 #default: 0.8
pot_ctrl_kv: 2.5 #damping default: 1.0
pot_ctrl_kp: 3.0 #stiffness default: 2.0
pot_ctrl_virt_mass: 0.5 #default: 0.8

#Parameters specifying collision velocity filter 
influence_radius: 2.0 #[m] distance from robot_center
obstacle_damping_dist: 1.0 # used as slow-down dist
stop_threshold: 0.1 #[m]
use_circumscribed_threshold: 0.2 #[rad/s]
```
