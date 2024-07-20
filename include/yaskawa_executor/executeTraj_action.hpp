#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaskawa_executor/action/traj_executor.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <fstream>
#include <iostream>
#include <motoros2_interfaces/srv/queue_traj_point.hpp>
#include <rclcpp/qos.hpp>

using TrajExecutor = yaskawa_executor::action::TrajExecutor;     // our action (we're the server here)
using QueueTrajPoint = motoros2_interfaces::srv::QueueTrajPoint; // client for the Yaskawa QTP service actually

std::string ResultComment(int type)
{
  switch (type)
  {
  case TrajExecutor::Result::NOT_SET:
    return "response not set, unexpected output";
  case TrajExecutor::Result::SUCCESS:
    return "Success, trajectory executed";

  case TrajExecutor::Result::EMPTY_TRAJ_MSG:
    return "Joint_trajectory is empty";
  case TrajExecutor::Result::EMPTY_JOINTS_IN_TRAJ_MSG:
    return "Points in joint_trajectory do not contain joint positions or velocities";
  case TrajExecutor::Result::JOINT_STATES_NOT_AVAILABLE:
    return "Can't access the /joint_states topic";
  case TrajExecutor::Result::JOINT_NAMES_DONT_MATCH:
    return "Joint_trajectory and /joint_states don't have matching joint names";
  case TrajExecutor::Result::TOO_FAR_FROM_FIRST_POINT:
    return "Current robot configuration is too far from first point on trajectory";
  case TrajExecutor::Result::LAST_VELOCITIES_NOT_ZERO:
    return "Last point velocities must be set to zero";

  case TrajExecutor::Result::ABORT_DUE_TO_NEW_GOAL:
    return "Aborting because a new goal has been received!";
  case TrajExecutor::Result::IS_CANCELLED:
    return "This goal has been cancelled!";

  case TrajExecutor::Result::CANT_CONNECT_TO_QTP:
    return "Timed out while waiting on the QTP";
  case TrajExecutor::Result::QTP_MAX_RETRIES_REACHED:
    return "Reached the max retries before finishing current point";
  case TrajExecutor::Result::QTP_FAILED:
    return "QTP got an error shown in result_code";
  case TrajExecutor::Result::QTP_UNRESPONSIVE:
    return "QTP became unresponsive and request timed out";
  default:
    return "unexpected result code";
  }
}

class TrajectoryExecutionServer : public rclcpp::Node
{
public:
  using ServerGoalHandleTE = rclcpp_action::ServerGoalHandle<TrajExecutor>;

  // constructor
  explicit TrajectoryExecutionServer();

  // destructor
  ~TrajectoryExecutionServer();

private:
  rclcpp_action::Server<TrajExecutor>::SharedPtr action_server_;
  rclcpp::Client<QueueTrajPoint>::SharedPtr client_qtp_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Parameter param_msg_age_, param_init_deviation_, param_init_time_, param_max_retries_, param_busy_wait_, param_conv_threshold_;

  std::vector<std::string> desired_joint_names_; // desired_joint_names_ is going to be filled with the correct joint names from joint_states_.
  int result_code_;
  int action_result_code_;
  unsigned int points_in_trajectory_;
  bool moving_, received_joint_state_, joint_states_are_correct_;
  std::vector<double> filtered_joint_positions_;
  sensor_msgs::msg::JointState saved_joint_states_;
  double max_allowed_msg_age_ = 0.05;           // max allowed age of the joint states message to make sure that it is recent, in seconds (changed via params)
  double max_allowed_initial_deviation_ = 0.02; // 0.02rad ~1.15degrees total on all axes combined. If more than this, the server will refuse the trajectory
  bool add_initial_point_ = false;              // the code decides based on the deviation
  double time_for_initial_adjustment_ = 1.0;    // give some time to adjust the initial position (it's a ros param)
  std::shared_ptr<ServerGoalHandleTE> server_goal_handle_, recorded_goal_handle_;
  trajectory_msgs::msg::JointTrajectory recorded_trajectory_;
  // streaming parameters:
  int max_retries_ = 2000;              // try to send each point to Yaskawa this amount of times
  double busy_wait_time_ = 0.005;       // sleep duration between attempts to send a point
  double convergence_threshold_ = 0.01; // used for the last point of a trajectory, compares current state (joint_states msg) with the last point of a trajectory.

  bool new_goal_received_ = false;
  std::thread trajectory_execution_thread_;

  // callback for when our server receives a new goal
  rclcpp_action::GoalResponse handle_received_goal(const rclcpp_action::GoalUUID &uuid, const std::shared_ptr<const TrajExecutor::Goal> goal);

  // callback for when our server receives a cancel request
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<ServerGoalHandleTE> goal_handle);

  // callback for when the server's goal is accepted (all are)
  void handle_accepted_goal(const std::shared_ptr<ServerGoalHandleTE> goal_handle);

  // the actual execution of goal (preparing everything to call yaskawa actions)
  void execute(const std::shared_ptr<ServerGoalHandleTE> goal_handle);

  // validation of all incoming goal variables
  bool validate_goal(const std::shared_ptr<const TrajExecutor::Goal> goal);

  // prepare result in case of failure
  void return_failure(const std::shared_ptr<ServerGoalHandleTE> goal_handle);

  // prepare result in case of cancellation
  void return_cancelled(const std::shared_ptr<ServerGoalHandleTE> goal_handle);

  // prepare result in case of success
  void return_success(const std::shared_ptr<ServerGoalHandleTE> goal_handle);

  // preparation of trajectory in case there is an initial point
  void prepare_the_trajectory();

  // sending yaskawa QTP service request
  int send_point_to_QTP(trajectory_msgs::msg::JointTrajectoryPoint &current_point);

  // joint state callback to verify the initial position & to make sure that the robot has stopped
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr joint_states);

  // check timestamp on the joint_states message
  bool is_joint_state_fresh();

  // compute joint distance between two points
  double compute_joint_dist_sum_wrt_joint_states(const trajectory_msgs::msg::JointTrajectoryPoint &joint_trajectory_point);

  // check timestamp on the joint_states message
  void filter_joint_positions();
};
