
# `Yaskawa_Executor`
## Overview
`Yaskawa_Executor` is a ROS 2 package that is an action server for sending longer trajectories to a Yaskawa (motoros2). Behind the scenes it is also a client for the QueueTrajPoint service provided by motoros2. It takes a trajectory, does thorough checks on it and sends it to the QueueTrajPoint point by point.

## Action Interface
The action server in this package uses the `TrajExecutor.action` action interface:
- **Goal**: `trajectory_msgs/JointTrajectory joint_trajectory`
- **Feedback**:
	- `bool moving`
	- `int32 current_point #current point SENT to QTP`
	- `int32 total_points`
- **Result**: 
	- `int32 result_code`
	- `string result_comment`

  ## Dependencies
This package depends on the following ROS 2 packages:
- `rclcpp`
- `rclcpp_action`
- `trajectory_msgs`
- `motoros2_interfaces`
- `control_msgs`
- `sensor_msgs`
- `action_msgs`

## Usage
### Running the Action Server
To run the action server, use the provided launch file:
```bash
ros2 launch yaskawa_executor traj_action_launch.py
```

### Parameters
The action server can be configured using the following parameters:
- `max_joint_msg_age` (default: 0.05): Maximum age of last joint states message, in seconds.
- `max_initial_deviation_rad` (default: 0.02): Maximum initial deviation in radians (comparing current joint positions and the first point on the trajectory).
- `time_for_initial_adjustment` (default: 1.0): Time allocated for initial adjustment, in seconds.
- `max_retries` (default: 2000): Maximum number of retries for sending each point. Set to 0 for infinite retries.
- `busy_wait_time` (default: 0.005): Wait time between retries, in seconds.
- `convergence_threshold` (default: 0.01): Convergence threshold in radians for considering the trajectory completed (only used for the last point).


## TODO
- add checking for distance between current `/joint_states` and the desired point (from trajectory) - if it is too big (what is too big???) - abort! Example: this can happen when the robot does a motion waaay too fast and then it somehow never manages to catch up with the desired states
