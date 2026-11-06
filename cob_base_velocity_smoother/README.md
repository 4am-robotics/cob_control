The
cob_base_velocity_smoother
package provides two implementations for a velocity smoother that both
read velocity messages (geometry_msgs::Twist) and then publish messages
of the same type for "smoothed" velocity to avoid jerky behavior.

The two nodes available are:

- `cob_base_velocity_smoother`
- `velocity_smoother`

## Hardware Requirements

To use this package you either need a real or a simulated Care-O-Bot
(see [cob_bringup](https://index.ros.org/p/cob_bringup) and
[cob_bringup_sim](https://index.ros.org/p/cob_bringup_sim)
respectively).

## ROS API

The following states the ROS API for the `cob_base_velocity_smoother`
node.

### cob_base_velocity_smoother

The `cob_base_velocity_smoother` node takes in
[geometry_msgs/Twist](http://ros.org/doc/api/geometry_msgs/html/msg/Twist.html)
messages from the navigation or teleoperation device and publishes
recalculated
[geometry_msgs/Twist](http://ros.org/doc/api/geometry_msgs/html/msg/Twist.html)
messages to avoid jerky behavior.

#### Subscribed Topics

- `input`
  ([geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)):
  movement command published by the navigation or teleoperation device.

#### Published Topics

- `output`
  ([geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)):
  recalculated movement command

#### Parameters

- `~circular_buffer_capacity` (`int`, default:
  `12 for cob3, 6 for raw3`): Specifies the capacity of the internal
  circular buffer (number of saved velocity messages).
- `~maximal_time_delay` (`double`, default: `4.0 [seconds]`): Specifies
  the maximal time delay for saved messages (older messages will be
  deleted).
- `~thresh_max_acc` (`double`, default: `0.02 [m/s^2]`): Specifies the
  threshold for the maximal allowed acceleration.
- `~loop_rate` (`double`, default: `30.0 [Hz]`): Specifies the ros loop
  rate of the velocity smoother.
- `~min_input_rate` (`double`, default: `9.0 [Hz]`): Specifies the
  minimal input rate that is expected before filling the buffer with
  zeros.
- `~maximal_time_delay_to_stop` (`double`, default: `0.1 [seconds]`):
  Specifies the maximal time-delay until stopping the robot when there
  are no incoming messages.

## Usage

All hardware configuration is done in the
[cob_hardware_config](https://index.ros.org/p/cob_hardware_config)
package. Parameter files for both nodes have to be placed in
"cob_hardware_config/\$(env ROBOT)/config/" for each ROBOT.
