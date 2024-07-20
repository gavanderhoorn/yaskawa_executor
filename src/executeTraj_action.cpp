#include <yaskawa_executor/executeTraj_action.hpp>

TrajectoryExecutionServer::TrajectoryExecutionServer()
    : Node("trajectory_action_server")
{
  // declare the server
  this->action_server_ = rclcpp_action::create_server<TrajExecutor>(
      this,
      "/trajectory_action_server",
      std::bind(&TrajectoryExecutionServer::handle_received_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryExecutionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&TrajectoryExecutionServer::handle_accepted_goal, this, std::placeholders::_1));

  // subscribe to joint states
  auto custom_qos_profile = rclcpp::QoS(rclcpp::SensorDataQoS());
  joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", custom_qos_profile, std::bind(&TrajectoryExecutionServer::joint_states_callback, this, std::placeholders::_1));
  received_joint_state_ = false;
  joint_states_are_correct_ = false;

  // declare the client for QTP
  this->client_qtp_ = this->create_client<QueueTrajPoint>("queue_traj_point");

  // declare the parameters:
  this->declare_parameter("max_joint_msg_age", 0.05);          // how old can the joint_states message be, in seconds
  this->declare_parameter("max_initial_deviation_rad", 0.02);  // sum of deviation on all axes has to be below this
  this->declare_parameter("time_for_initial_adjustment", 1.0); // seconds allocated to fix initial deviation
  this->declare_parameter("max_retries", 2000);                // try to send each point 2000 times, if max_retries=0, then the server will try forever
  this->declare_parameter("busy_wait_time", 0.005);            // wait time (in seconds) between retries
  this->declare_parameter("convergence_threshold", 0.01);      // at this distance (sum of all axes in radians) we say that the trajectory is completed and return a SUCCESS

  // get the updated param values:
  this->get_parameter("max_joint_msg_age", param_msg_age_);
  this->get_parameter("max_initial_deviation_rad", param_init_deviation_);
  this->get_parameter("time_for_initial_adjustment", param_init_time_);
  this->get_parameter("max_retries", param_max_retries_);
  this->get_parameter("busy_wait_time", param_busy_wait_);
  this->get_parameter("convergence_threshold", param_conv_threshold_);

  // fill the variables with param values:
  max_allowed_msg_age_ = param_msg_age_.as_double();
  max_allowed_initial_deviation_ = param_init_deviation_.as_double();
  time_for_initial_adjustment_ = param_init_time_.as_double();
  max_retries_ = param_max_retries_.as_int();
  busy_wait_time_ = param_busy_wait_.as_double();
  convergence_threshold_ = param_conv_threshold_.as_double();

  // we record a goal handle later to deal with concurrent calls to the server
  recorded_goal_handle_ = nullptr;
}

TrajectoryExecutionServer::~TrajectoryExecutionServer()
{
  // Clean up the trajectory execution thread
  if (trajectory_execution_thread_.joinable())
  {
    trajectory_execution_thread_.join();
  }
}

rclcpp_action::GoalResponse TrajectoryExecutionServer::handle_received_goal(const rclcpp_action::GoalUUID &uuid, const std::shared_ptr<const TrajExecutor::Goal> goal)
{
  // avoid 'unused variable' warnings!
  (void)uuid;
  (void)goal;

  // accept all goals and abort them later with communication to client
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

rclcpp_action::CancelResponse TrajectoryExecutionServer::handle_cancel(const std::shared_ptr<ServerGoalHandleTE> goal_handle)
{
  // avoid 'unused variable' warnings!
  (void)goal_handle;

  // Log the receipt of the cancel request
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  // accept all cancel requests
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryExecutionServer::handle_accepted_goal(const std::shared_ptr<ServerGoalHandleTE> goal_handle)
{
  // save the accepted goal handle for later use
  server_goal_handle_ = goal_handle;

  // initialize values
  result_code_ = TrajExecutor::Result::NOT_SET;
  action_result_code_ = -1;

  // check if there is a recorded goal handle. If yes & different from current, abort the recorded goal.
  if (recorded_goal_handle_ != nullptr && recorded_goal_handle_ != goal_handle)
  {
    new_goal_received_ = true;
  }
  else
  {
    new_goal_received_ = false;
  }
  recorded_goal_handle_ = goal_handle;

  // check if the QTP is available
  if (!client_qtp_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(this->get_logger(), "QTP not available after waiting");
    result_code_ = TrajExecutor::Result::CANT_CONNECT_TO_QTP;
    return_failure(goal_handle);
    return;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Connection established with QTP");
  }

  // Validate that the trajectory request has been correctly filled
  if (!validate_goal(goal_handle->get_goal()))
  {
    return_failure(goal_handle);
    return;
  }

  // Spawn a new thread for trajectory execution
  std::thread trajectory_execution_thread([this, goal_handle]()
                                          {
      if(new_goal_received_){
          rclcpp::sleep_for(std::chrono::milliseconds(static_cast<long int>(1*1000)));
      }
      execute(goal_handle); });

  // Detach the trajectory execution thread
  trajectory_execution_thread.detach();
}

void TrajectoryExecutionServer::execute(const std::shared_ptr<ServerGoalHandleTE> goal_handle)
{
  // get the goal
  // const auto goal = goal_handle->get_goal();
  recorded_trajectory_ = goal_handle->get_goal()->joint_trajectory;
  if (add_initial_point_)
  {
    prepare_the_trajectory();
  }
  points_in_trajectory_ = recorded_trajectory_.points.size();

  // send it to QTP
  auto feedback = std::make_shared<TrajExecutor::Feedback>();
  feedback->total_points = points_in_trajectory_;
  int result = -99999;
  for (unsigned int i = 0; i < points_in_trajectory_; i++)
  {
    // check for cancellation
    if (goal_handle->is_canceling())
    {
      result_code_ = TrajExecutor::Result::IS_CANCELLED;
      return_cancelled(goal_handle);
      return;
    }
    if (new_goal_received_)
    {
      RCLCPP_INFO(this->get_logger(), "New goal received! Aborting current goal!");
      new_goal_received_ = false;
      // recorded_goal_handle_ = nullptr;
      result_code_ = TrajExecutor::Result::ABORT_DUE_TO_NEW_GOAL;
      return_failure(goal_handle);
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "queueing pt %d", i);
    result = send_point_to_QTP(recorded_trajectory_.points[i]);
    feedback->moving = true;
    feedback->current_point = i + 1; // because i starts from 0
    goal_handle->publish_feedback(feedback);

    // If this is an error or still BUSY, something is wrong. Abort the goal and report error
    std::string error_string = "";
    switch (result)
    {
    case motoros2_interfaces::msg::QueueResultEnum::SUCCESS:
      RCLCPP_DEBUG(this->get_logger(), "successfully queued pt %d", i);
      break;
    case motoros2_interfaces::msg::QueueResultEnum::BUSY:
      // still busy, meaning we reached max retries before finishing the current point
      error_string = "reached max retries = " + std::to_string(max_retries_) + ", aborting goal ";
      RCLCPP_ERROR(this->get_logger(), error_string.c_str());
      result_code_ = TrajExecutor::Result::QTP_MAX_RETRIES_REACHED;
      return_failure(goal_handle);
      return;
    case motoros2_interfaces::msg::QueueResultEnum::UNABLE_TO_PROCESS_POINT:
      // the QTP became unresponsive (crashed?)
      error_string = "no response from QTP, timeout reached, aborting goal ";
      RCLCPP_ERROR(this->get_logger(), error_string.c_str());
      result_code_ = TrajExecutor::Result::QTP_UNRESPONSIVE;
      return_failure(goal_handle);
      return;
    default: // any other error
      error_string = "failed to queue pt " + std::to_string(i) +
                     ", aborting goal (queue server reported: " + std::to_string(result) + ")";
      RCLCPP_ERROR(this->get_logger(), error_string.c_str());
      result_code_ = TrajExecutor::Result::QTP_FAILED;
      action_result_code_ = result;

      return_failure(goal_handle);
      return;
    }
    // if (result != motoros2_interfaces::msg::QueueResultEnum::SUCCESS)
    // {
    //   if (result == motoros2_interfaces::msg::QueueResultEnum::BUSY)
    //   {
    //     // still busy, meaning we reached max retries before finishing the current point
    //     std::string error_string = "reached max retries = " + std::to_string(max_retries_) + ", aborting goal ";
    //     RCLCPP_ERROR(this->get_logger(), error_string.c_str());
    //     result_code_ = TrajExecutor::Result::QTP_MAX_RETRIES_REACHED;

    //     return_failure(goal_handle);
    //     return;
    //   }
    //   else
    //   {
    //     std::string error_string = "failed to queue pt " + std::to_string(i) +
    //                                ", aborting goal (queue server reported: " + std::to_string(result) + ")";
    //     RCLCPP_ERROR(this->get_logger(), error_string.c_str());
    //     result_code_ = TrajExecutor::Result::QTP_FAILED;
    //     action_result_code_ = result;

    //     return_failure(goal_handle);
    //     return;
    //   }
    // }
    // else
    // {
    //   RCLCPP_DEBUG(this->get_logger(), "successfully queued pt %d", i);
    // }
  }
  RCLCPP_INFO(this->get_logger(), "successfully queued all pts( %u )", points_in_trajectory_);

  RCLCPP_INFO(this->get_logger(), "waiting for robot to reach final traj pt (threshold: %f rad)", convergence_threshold_);

  if (!is_joint_state_fresh())
  {
    RCLCPP_WARN(this->get_logger(), "Can't track progress as no joint states received, not waiting for execution before reporting success");
  }
  else
  {
    double curr_dist;
    rclcpp::Time start_time = now();
    rclcpp::Duration elapsed_time = now() - start_time;
    rclcpp::Duration time1 = recorded_trajectory_.points.back().time_from_start;
    rclcpp::Duration allocated_time = time1 - recorded_trajectory_.points[recorded_trajectory_.points.size() - 2].time_from_start;
    // doing this directly throws an error for some reason:
    // rclcpp::Duration toto = (recorded_trajectory_.points.back().time_from_start) - (recorded_trajectory_.points[recorded_trajectory_.points.size()-2].time_from_start);
    while (rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<long int>(busy_wait_time_ * 1000)));
      filter_joint_positions(); // update the vector of filtered joint positions!
      curr_dist = compute_joint_dist_sum_wrt_joint_states(recorded_trajectory_.points.back());
      if (curr_dist >= convergence_threshold_)
      {
        RCLCPP_DEBUG(this->get_logger(), "current distance =  %f ", curr_dist);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<long int>(busy_wait_time_ * 1000)));
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "reached final traj pt (distance: %f)", curr_dist);
        break;
      }
      elapsed_time = now() - start_time;
      if (elapsed_time >= allocated_time * 2)
      {
        // we allow for 2x allocated time for the last point just in case
        RCLCPP_WARN(this->get_logger(), "timed out, assuming success");
        break;
      }
    }
  }
  result_code_ = TrajExecutor::Result::SUCCESS;
  return_success(server_goal_handle_);
}

bool TrajectoryExecutionServer::validate_goal(const std::shared_ptr<const TrajExecutor::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Attempting goal validation now");

  if (received_joint_state_)
  {
    if (is_joint_state_fresh())
    {
      RCLCPP_DEBUG(this->get_logger(), "Got a message from joint_states");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Last message from joint_states is more than 0.05s old.");
      result_code_ = TrajExecutor::Result::JOINT_STATES_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Can't access joint_states, not safe to continue!");
    result_code_ = TrajExecutor::Result::JOINT_STATES_NOT_AVAILABLE;
    return false;
  }

  // trajectory must be supplied in joint_trajectory
  if (goal->joint_trajectory.points.empty())
  {
    // no trajectory
    RCLCPP_ERROR(this->get_logger(), "the supplied joint_trajectory message is empty!");
    result_code_ = TrajExecutor::Result::EMPTY_TRAJ_MSG;
    return false;
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "joint_trajectory contains points");
    if (goal->joint_trajectory.points[0].positions.empty() || goal->joint_trajectory.points[0].velocities.empty())
    {
      // trajectory technically there, but the positions or velocities are not set
      RCLCPP_ERROR(this->get_logger(), "Empty positions or velocities in the trajectory");
      result_code_ = TrajExecutor::Result::EMPTY_JOINTS_IN_TRAJ_MSG;
      return false;
    }
    else if (std::accumulate(goal->joint_trajectory.points.back().velocities.begin(), goal->joint_trajectory.points.back().velocities.end(), 0.0) > 0.001)
    {
      // last point has non-zero velocities!
      RCLCPP_ERROR(this->get_logger(), "Last point of the trajectory must have velocity=0.0");
      result_code_ = TrajExecutor::Result::LAST_VELOCITIES_NOT_ZERO;
      return false;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "joint_trajectory.points is correctly formatted");
    }

    desired_joint_names_ = goal->joint_trajectory.joint_names; // record the joint names from the goal
    filter_joint_positions();                                  // filter the joint positions from /joint_states to be in the same order as the goal
    // If the loop completed successfully, set joint_states_are_correct_ to true
    if (filtered_joint_positions_.size() == desired_joint_names_.size())
    {
      joint_states_are_correct_ = true;
      RCLCPP_DEBUG(this->get_logger(), "joint_trajectory.joint_names format matches with /joint_states.name");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "joint_trajectory.joint_names format does not match with /joint_states.name. Cannot verify initial robot position.");
      result_code_ = TrajExecutor::Result::JOINT_NAMES_DONT_MATCH;
      return false;
    }

    double joint_distance = compute_joint_dist_sum_wrt_joint_states(goal->joint_trajectory.points[0]);
    if (joint_distance > max_allowed_initial_deviation_)
    {
      RCLCPP_ERROR(this->get_logger(), "current joint position too far from first point on trajectory!");
      result_code_ = TrajExecutor::Result::TOO_FAR_FROM_FIRST_POINT;
      return false;
    }
    else if (joint_distance <= max_allowed_initial_deviation_ && joint_distance > 0.005)
    { // filter out anything below 0.005rad ~ 0.25degrees is close enough
      RCLCPP_WARN(this->get_logger(), "current joint position has a distance of %f rad from first point on trajectory, will add initial point to trajectory!", joint_distance);
      add_initial_point_ = true;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "current joint position matches first point on trajectory");
      add_initial_point_ = false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "The goal trajectory is validated");

  return true;
}

void TrajectoryExecutionServer::return_failure(const std::shared_ptr<ServerGoalHandleTE> goal_handle)
{
  if (result_code_ != TrajExecutor::Result::ABORT_DUE_TO_NEW_GOAL)
  {
    recorded_goal_handle_ = nullptr;
  }
  RCLCPP_INFO(this->get_logger(), "Handling Failure");
  auto own_result = std::make_shared<TrajExecutor::Result>();
  if (result_code_ == TrajExecutor::Result::QTP_FAILED)
  {
    // use the action result code instead
    own_result->result_code = action_result_code_;
  }
  else
  {
    own_result->result_code = result_code_;
  }
  own_result->result_comment = ResultComment(result_code_);
  RCLCPP_DEBUG(this->get_logger(), "sending out code=%i", own_result->result_code);
  RCLCPP_DEBUG(this->get_logger(), "corresponding string==%s", own_result->result_comment.c_str());
  goal_handle->abort(own_result);
  // return;
}

void TrajectoryExecutionServer::return_cancelled(const std::shared_ptr<ServerGoalHandleTE> goal_handle)
{
  recorded_goal_handle_ = nullptr;
  RCLCPP_INFO(this->get_logger(), "Handling Cancellation");
  auto own_result = std::make_shared<TrajExecutor::Result>();
  own_result->result_code = result_code_;
  own_result->result_comment = ResultComment(result_code_);
  goal_handle->canceled(own_result);
  // return;
}

void TrajectoryExecutionServer::return_success(const std::shared_ptr<ServerGoalHandleTE> goal_handle)
{
  recorded_goal_handle_ = nullptr;
  RCLCPP_INFO(this->get_logger(), "Handling Success");
  auto own_result = std::make_shared<TrajExecutor::Result>();
  own_result->result_code = result_code_;
  own_result->result_comment = ResultComment(result_code_);
  goal_handle->succeed(own_result);
  // return;
}

void TrajectoryExecutionServer::prepare_the_trajectory()
{
  // during goal check we discovered a difference between the current joint states saved in filtered_joint_positions_ and the first point of trajectory, but the difference is not too large
  // we create a new vector with the initial point & recompute all the other timestamps
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> all_points;
  all_points.resize(0);

  trajectory_msgs::msg::JointTrajectoryPoint initial_point;
  initial_point.positions = filtered_joint_positions_;
  initial_point.velocities.resize(initial_point.positions.size(), 0.0); // we want to stop after the initial adjustment!
  initial_point.time_from_start = rclcpp::Duration::from_seconds(time_for_initial_adjustment_);
  // if problems - use this: rclcpp::Duration from_start(std::chrono::nanoseconds(static_cast<int64_t>(time_init_config_to_first_waypoint * 1e9)))

  all_points.push_back(initial_point);

  // fill the vector from the trajectory message
  // cannot use this because I need to amend the time_from_start for each point!
  // all_points.insert(all_points.end(), trajectory_msg.points.begin(), trajectory_msg.points.end());
  for (unsigned i = 0; i < recorded_trajectory_.points.size(); i++)
  {

    all_points.push_back(recorded_trajectory_.points[i]);
    // in this sum sequence it throws an error (similarly as all_points.back().time_from_start += rclcpp::Duration::from_seconds(time_to_approach)) because the operators + and += are not defined for the message, yet are defined for rclcpp::Duration, hence the uncommented version works
    // all_points.back().time_from_start = all_points.back().time_from_start + rclcpp::Duration::from_seconds(time_to_approach);
    all_points.back().time_from_start = rclcpp::Duration::from_seconds(time_for_initial_adjustment_) + all_points.back().time_from_start;
  }

  recorded_trajectory_.points.clear();
  recorded_trajectory_.points = all_points;
}

int TrajectoryExecutionServer::send_point_to_QTP(trajectory_msgs::msg::JointTrajectoryPoint &current_point)
{
  RCLCPP_DEBUG(this->get_logger(), "attempting to queue pt (max_retries: %d)", max_retries_);
  int attempt = 0;
  int result_code = -99999;
  QueueTrajPoint::Request::SharedPtr req = std::make_shared<QueueTrajPoint::Request>();
  req->joint_names = recorded_trajectory_.joint_names;
  req->point = current_point;

  while (rclcpp::ok() && ((attempt < max_retries_) || (max_retries_ == 0)))
  {
    // loops while attempts<max_retries OR forever if max_retries=0
    RCLCPP_DEBUG(this->get_logger(), "queueing pt (attempt: %d)", attempt);
    auto response = client_qtp_->async_send_request(req);

    auto timeout = std::chrono::seconds(10);
    if (response.wait_for(timeout) == std::future_status::timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Request timed out during attempt: %d. Exiting.", attempt);
      result_code = motoros2_interfaces::msg::QueueResultEnum::UNABLE_TO_PROCESS_POINT;
      break;
    }
    // only if we receive a BUSY response we try again. Anything else
    // is something only the caller can handle (including OK)
    auto result_stuff = response.get();
    auto rescode = result_stuff->result_code;
    auto resmsg = result_stuff->message;
    result_code = rescode.value;
    if (result_code == motoros2_interfaces::msg::QueueResultEnum::BUSY)
    {
      RCLCPP_DEBUG(this->get_logger(), "Busy (attempt: %d). Trying again later.", attempt);
      attempt += 1;
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<long int>(busy_wait_time_ * 1000)));
      continue;
    }
    if (result_code == motoros2_interfaces::msg::QueueResultEnum::SUCCESS)
    {
      RCLCPP_DEBUG(this->get_logger(), "queue server accepted point.");
      break;
    }

    // anything else is an error, so report
    RCLCPP_WARN(this->get_logger(), "queue server returned error: (%d)", result_code);
    if (!resmsg.empty())
    {
      RCLCPP_WARN(this->get_logger(), "error message: '%s'", resmsg.c_str());
      break;
    }
  }

  if (attempt == max_retries_)
  {
    // reached max retries, stopping the execution of this trajectory
    result_code = motoros2_interfaces::msg::QueueResultEnum::BUSY; // setting just in case, I think it should already be like this
  }

  // either -1, or one of the values from QueueResultEnum
  return result_code;
}

void TrajectoryExecutionServer::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr joint_states)
{
  received_joint_state_ = true;
  saved_joint_states_ = *joint_states;
}

bool TrajectoryExecutionServer::is_joint_state_fresh()
{
  // Compare timestamps to check the age of the joint state
  rclcpp::Time current_time = now();
  rclcpp::Time msg_time(saved_joint_states_.header.stamp);
  rclcpp::Duration age = current_time - msg_time;
  return age.seconds() <= max_allowed_msg_age_;
}

double TrajectoryExecutionServer::compute_joint_dist_sum_wrt_joint_states(const trajectory_msgs::msg::JointTrajectoryPoint &joint_trajectory_point)
{

  const auto &joint_positions2 = joint_trajectory_point.positions;
  double distance = 0.0;

  for (size_t i = 0; i < filtered_joint_positions_.size(); ++i)
  {
    double position_diff = std::abs(filtered_joint_positions_[i] - joint_positions2[i]);
    distance += position_diff;
  }

  return distance;
}

void TrajectoryExecutionServer::filter_joint_positions()
{
  filtered_joint_positions_.clear();
  for (const auto &goal_names : desired_joint_names_)
  {
    auto it = std::find(saved_joint_states_.name.begin(), saved_joint_states_.name.end(), goal_names);
    if (it == saved_joint_states_.name.end())
    {
      joint_states_are_correct_ = false; // Joint name not found, abort
      break;
    }

    // Find the index of the joint in joint_states and add its position to joint_positions_
    size_t index = std::distance(saved_joint_states_.name.begin(), it);
    filtered_joint_positions_.push_back(saved_joint_states_.position[index]);
  }
}

/*
  -------------------------------------------------------- main --------------------------------------------------------------------
*/

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // create the node
  auto node = std::make_shared<TrajectoryExecutionServer>();

  // create the executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  // Start the executor in a separate thread
  std::thread executor_thread([executor]()
                              { executor->spin(); });

  // Wait for the executor thread to finish
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}