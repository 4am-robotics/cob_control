The `cob_footprint_observer` observes the setup of the Care-O-Bot
(including arm and tray) and inflates the footprint to contain the arm
and the tray. It allows for reading the adjusted footprint using either
the `GetFootprint` service or the published `adjusted_footprint`
topic.

**Note:** The `cob_footprint_observer` only works with rectangular
footprints.

## Hardware Requirements

To use this package you need either need a real or a simulated
Care-O-Bot (see [cob_bringup](https://index.ros.org/p/cob_bringup) and
[cob_bringup_sim](https://index.ros.org/p/cob_bringup_sim)
respectively).

## ROS API

The cob_footprint_observer
package provides a configurable node for checking and adjusting the
footprint based on the current setup.

### cob_footprint_observer

The `cob_footprint_observer` node uses a `tf::TransformListener` to
check the transformations for desired frames and adjusts the footprint
accordingly. It publishes the adjusted footprint as a [topic](Topics)
and sets the footprint using a [service](Services).

#### Published Topics

- `adjusted_footprint`
  ([geometry_msgs/msg/PolygonStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PolygonStamped.html)):
  Publishes the adjusted footprint of the robot.

#### Services

- `/get_footprint`
  ([cob_footprint_observer/srv/GetFootprint](https://docs.ros.org/en/rolling/p/cob_footprint_observer/srv/GetFootprint.html)):
  Responds with the adjusted footprint.

#### Parameters

- `~footprint_source` (`string`, default:
  `"/local_costmap_node/costmap"`): Specifies which node to search for
  the initial footprint, e.g. /local_costmap_node/costmap.
- `~frames_to_check` (`string`, default: `""`): Frames which should be
  checked for footprint adjustment (separated by spaces), e.g.
  /base_link /sdh_tip_link /arm_4_link.
- `~robot_base_frame` (`string`, default: `"/base_link"`): Specifies the
  robot base frame, e.g. /base_link.

## Usage/Examples

This package is not intended to be used directly, but with the
corresponding launch and yaml files. For starting the footprint observer
with the `collision_velocity_filter` use 

```
roslaunch cob_bringup base_collision_observer.launch
```

 For
including the footprint observer in your overall launch file use


```
<!-- start footprint observer node -->
<node pkg="cob_footprint_observer" type="footprint_observer" name="footprint_observer" output="screen">
  <!-- load parameter file -->
  <rosparam file="$(find cob_hardware_config)/$(env ROBOT)/config/footprint_observer_params.yaml" command="load" />
</node>
```

 A sample parameter file could look like this


```
# node from which initial footprint is read
footprint_source: /local_costmap_node/costmap

# 
robot_base_frame: /base_link
# frames to check for footprint adjustment
frames_to_check: /sdh_tip_link /arm_6_link /arm_4_link
```
