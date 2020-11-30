#include <mav_msgs/default_topics.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <std_msgs/Int32.h>
#include "mav_local_planner/mav_local_planner.h"
#include "mav_local_planner/color.h"
namespace mav_planning
{

  MavLocalPlanner::MavLocalPlanner(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        command_publishing_spinner_(4, &command_publishing_queue_),
        planning_spinner_(4, &planning_queue_),
        verbose_(false),
        global_frame_id_("map"),
        local_frame_id_("odom"),
        mpc_prediction_horizon_(300),
        command_publishing_dt_(1.0),
        replan_dt_(1.0),
        replan_lookahead_sec_(0.1),
        avoid_collisions_(true),
        autostart_(true),
        plan_to_start_(true),
        smoother_name_("loco"),
        current_waypoint_(-1),
        path_index_(0),
        max_failures_(5),
        num_failures_(0),
        num_aborts_(0),
        esdf_server_(nh_, nh_private_),
        loco_planner_(nh_, nh_private_)
  {
    // Set up some settings.
    constraints_.setParametersFromRos(nh_private_);
    esdf_server_.setTraversabilityRadius(constraints_.robot_radius);
    loco_planner_.setEsdfMap(esdf_server_.getEsdfMapPtr());
    goal_selector_.setParametersFromRos(nh_private_);
    //goal_selector_.setTsdfMap(esdf_server_.getTsdfMapPtr());

    nh_private_.param("verbose", verbose_, verbose_);
    nh_private_.param("global_frame_id", global_frame_id_, global_frame_id_);
    nh_private_.param("local_frame_id", local_frame_id_, local_frame_id_);
    nh_private_.param("mpc_prediction_horizon", mpc_prediction_horizon_,
                      mpc_prediction_horizon_);
    nh_private_.param("replan_dt", replan_dt_, replan_dt_);
    nh_private_.param("replan_lookahead_sec", replan_lookahead_sec_,
                      replan_lookahead_sec_);
    nh_private_.param("command_publishing_dt", command_publishing_dt_,
                      command_publishing_dt_);
    nh_private_.param("avoid_collisions", avoid_collisions_, avoid_collisions_);
    nh_private_.param("autostart", autostart_, autostart_);
    nh_private_.param("plan_to_start", plan_to_start_, plan_to_start_);
    nh_private_.param("smoother_name", smoother_name_, smoother_name_);

    // Publishers and subscribers.
    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                  &MavLocalPlanner::odometryCallback, this);
    waypoint_sub_ =
        nh_.subscribe("waypoint", 1000, &MavLocalPlanner::waypointCallback, this);
    waypoint_list_sub_ = nh_.subscribe(
        "waypoint_list", 1000, &MavLocalPlanner::waypointListCallback, this);

    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

    path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "local_path", 1, true);
    robot_radius_marker_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
        "robot_radius", 1, true);
    full_trajectory_pub_ =
        nh_private_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            "full_trajectory", 1, true);
    planner_status_pub_ = nh_private_.advertise<std_msgs::Int32>("planner_status", 1, true);
    planner_active_pub_ = nh_private_.advertise<std_msgs::Int32>("planner_active", 1, true);
    wp_free_pub_ = nh_private_.advertise<geometry_msgs::PoseArray>("wp_free", 1, true);

    // Services.
    start_srv_ = nh_private_.advertiseService(
        "start", &MavLocalPlanner::startCallback, this);
    pause_srv_ = nh_private_.advertiseService(
        "pause", &MavLocalPlanner::pauseCallback, this);
    stop_srv_ = nh_private_.advertiseService(
        "stop", &MavLocalPlanner::stopCallback, this);

    position_hold_client_ =
        nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");

    // Start the planning timer. Will no-op most cycles.
    ros::TimerOptions timer_options(
        ros::Duration(replan_dt_),
        boost::bind(&MavLocalPlanner::planningTimerCallback, this, _1),
        &planning_queue_);

    planning_timer_ = nh_.createTimer(timer_options);

    // Start the command publishing spinner.
    command_publishing_spinner_.start();
    planning_spinner_.start();

    // Set up yaw policy.
    yaw_policy_.setPhysicalConstraints(constraints_);
    yaw_policy_.setYawPolicy(YawPolicy::PolicyType::kAnticipateVelocityVector);

    // Set up smoothers.
    const double voxel_size = esdf_server_.getEsdfMapPtr()->voxel_size();

    // Straight-line smoother.
    ramp_smoother_.setParametersFromRos(nh_private_);

    // Poly smoother.
    poly_smoother_.setParametersFromRos(nh_private_);
    poly_smoother_.setMinCollisionCheckResolution(voxel_size);
    poly_smoother_.setMapDistanceCallback(
        std::bind(&MavLocalPlanner::getMapDistance, this, std::placeholders::_1));
    poly_smoother_.setOptimizeTime(true);
    poly_smoother_.setSplitAtCollisions(avoid_collisions_);

    // Loco smoother!
    loco_smoother_.setParametersFromRos(nh_private_);
    loco_smoother_.setMinCollisionCheckResolution(voxel_size);
    loco_smoother_.setDistanceAndGradientFunction(
        std::bind(&MavLocalPlanner::getMapDistanceAndGradient, this,
                  std::placeholders::_1, std::placeholders::_2));
    // loco_smoother_.setOptimizeTime(true);
    // loco_smoother_.setResampleTrajectory(true);
    // loco_smoother_.setResampleVisibility(true);
    // loco_smoother_.setNumSegments(5);


    ROS_INFO_STREAM("[Mav Local Planner__]smoother_name_  "<< smoother_name_);
    path_found_ = false;
    planner_active = 0;
    // ROS_INFO_STREAM("[Mav Local Planner__]goal_selector_strategy  "<< goal_selector_strategy_);
    


  }

  void MavLocalPlanner::odometryCallback(const nav_msgs::Odometry &msg)
  {
    mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
  }

  void MavLocalPlanner::waypointCallback(const geometry_msgs::PoseStamped &msg)
  {
    // Plan a path from the current position to the target pose stamped.
    ROS_INFO("[Mav Local Planner] Got a waypoint!");
    // Cancel any previous trajectory on getting a new one.
    clearTrajectory();

    mav_msgs::EigenTrajectoryPoint waypoint;
    eigenTrajectoryPointFromPoseMsg(msg, &waypoint);

    waypoints_.clear();
    waypoints_.push_back(waypoint);
    current_waypoint_ = 0;

    // Execute one planning step on main thread.
    planningStep();
    startPublishingCommands();
  }

  void MavLocalPlanner::waypointListCallback(
      const geometry_msgs::PoseArray &msg)
  {
    // Plan a path from the current position to the target pose stamped.
    ROS_INFO("[Mav Local Planner] Got a list of waypoints, %zu long!",
             msg.poses.size());
    // Cancel any previous trajectory on getting a new one.
    clearTrajectory();

    waypoints_.clear();

    for (const geometry_msgs::Pose &pose : msg.poses)
    {
      mav_msgs::EigenTrajectoryPoint waypoint;
      eigenTrajectoryPointFromPoseMsg(pose, &waypoint);
      waypoints_.push_back(waypoint);
    }
    current_waypoint_ = 0;

    // Execute one planning step on main thread.
    planningStep();
    startPublishingCommands();
  }

  void MavLocalPlanner::planningTimerCallback(const ros::TimerEvent &event)
  {
    // Wait on the condition variable from the publishing...
    if (should_replan_.wait_for(replan_dt_) && path_found_)
    {
      if (verbose_)
      {
        ROS_WARN(
            "[Mav Planning Timer] Difference between real and expected: %f Real: "
            "%f Expected: %f Now: %f",
            (event.current_real - event.current_expected).toSec(),
            event.current_real.toSec(), event.current_expected.toSec(),
            ros::Time::now().toSec());
      }

      planningStep();
    }
  }

  void MavLocalPlanner::planningStep()
  {
    // ROS_INFO(
    //     "[Mav Local Planner][Plan Step] Waypoint index: %zd Total waypoints: %zu",
    //     current_waypoint_, waypoints_.size());
    if (current_waypoint_ < 0 ||
        static_cast<int>(waypoints_.size()) <= current_waypoint_)
    {
      // This means that we probably planned to the end of the waypoints!

      // If we're done with sending waypoints, alllll good. Just quit.
      if (path_index_ >= path_queue_.size() || path_queue_.empty())
      {
        return;
      }

      // If we're not though, we should probably double check the trajectory!
    }

    mav_trajectory_generation::timing::MiniTimer timer;
    constexpr double kCloseToOdometry = 0.1;

    // First, easiest case: if we're not avoiding collisions, just use the
    // favorite path smoother. We only do this on the first planning call then
    // ignore all the rest.
    if (!avoid_collisions_)
    {
      mav_msgs::EigenTrajectoryPointVector waypoints;
      mav_msgs::EigenTrajectoryPoint current_point;
      current_point.position_W = odometry_.position_W;
      current_point.orientation_W_B = odometry_.orientation_W_B;

      if (plan_to_start_)
      {
        waypoints.push_back(current_point);
      }
      waypoints.insert(waypoints.end(), waypoints_.begin(), waypoints_.end());

      mav_msgs::EigenTrajectoryPointVector path;

      if (planPathThroughWaypoints(waypoints, &path))
      {
        replacePath(path);
        current_waypoint_ = waypoints_.size();
      }
      else
      {
        ROS_INFO("[Mav Local Planner] Waypoint planning failed!");
      }
    }
    else if (path_queue_.empty())
    {
      // First check how many waypoints we haven't covered yet are in free space.
      mav_msgs::EigenTrajectoryPointVector free_waypoints;
      geometry_msgs::PoseArray free_waypoints_pose;
      // Do we need the odometry in here? Let's see.
      mav_msgs::EigenTrajectoryPoint current_point;

      
      current_point.position_W = odometry_.position_W;
      current_point.orientation_W_B = odometry_.orientation_W_B;

      // If the path doesn't ALREADY start near the odometry, the first waypoint
      // should be the current pose.
      int waypoints_added = 0;
      if (plan_to_start_ &&
          (current_point.position_W - waypoints_.front().position_W).norm() >
              kCloseToOdometry)
      {
        free_waypoints.push_back(current_point);
        waypoints_added = 1;
      }

      for (const mav_msgs::EigenTrajectoryPoint &waypoint : waypoints_)
      {
        if (getMapDistance(waypoint.position_W) < constraints_.robot_radius)
        {
          break;
        }
        free_waypoints.push_back(waypoint);
        geometry_msgs::PoseStamped pose;          
        mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(waypoint, &pose); 
        free_waypoints_pose.poses.push_back(pose.pose);
      }

      
      ROS_INFO("[Mav Local Planner] Of %zu waypoints, %zu are free.",
               waypoints_.size(), free_waypoints.size());
      bool success = false;
      if (free_waypoints.size() <= static_cast<size_t>(waypoints_added) ||
          free_waypoints.size() == 2)
      {
        // Okay whatever just search for the first waypoint.
        success = false;
      }
      else
      {
        // There is some hope! Maybe we can do path smoothing on these guys.
        mav_msgs::EigenTrajectoryPointVector path;
        success = planPathThroughWaypoints(free_waypoints, &path);
        if (success)
        {
          ROS_INFO(
              "[Mav Local Planner]  Successfully planned path through %zu free "
              "waypoints.",
              free_waypoints.size());
          success = isPathCollisionFree(path);
          if (success){
            wp_free_pub_.publish(free_waypoints_pose); 
          } 

          if (success)
          {
            replacePath(path);
            current_waypoint_ = std::min(free_waypoints.size() - waypoints_added,
                                         waypoints_.size() - 1);
            ROS_INFO(
                "[Mav Local Planner] Used smoothing through %zu waypoints! Total "
                "waypoint size: %zu, current point: %zd, added? %d",
                free_waypoints.size(), waypoints_.size(), current_waypoint_,
                waypoints_added);
            path_found_ = true;
          }
          else
          {
            ROS_WARN("[Mav Local Planner] But path was not collision free. :(");
            path_found_ = false;
          }
        }
      }
      // Give up!
      if (!success)
      {
        avoidCollisionsTowardWaypoint();
      }
    }
    else
    {
      // Otherwise let's just keep exploring.
      avoidCollisionsTowardWaypoint();
    }

    // ROS_INFO("[Mav Local Planner][Plan Step] Planning finished. Time taken: %f",
    //          timer.stop());
    visualizePath();
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    visualizeRobotRadius(current_point);
  }

  void MavLocalPlanner::avoidCollisionsTowardWaypoint()
  {
    if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size()))
    {
      return;
    }
    mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
    const double kCloseEnough = 0.05;

    const int64_t kDtNs =
        mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);

    if(verbose_){
        ROS_INFO_STREAM("[Mav Local Planner][Plan Step] Current odometry: "
                    << odometry_.position_W.transpose() << " Tracking waypoint ["
                    << current_waypoint_
                    << "]: " << waypoint.position_W.transpose());

    }

    // Save success and Trajectory.
    mav_trajectory_generation::Trajectory trajectory;
    bool success = false;

    if (!path_queue_.empty())
    {
      std::lock_guard<std::recursive_mutex> guard(path_mutex_);
      if(verbose_){
          ROS_INFO(
          "[Mav Local Planner][Plan Step] Trying to replan on existing path.");
      }

      mav_msgs::EigenTrajectoryPointVector path_chunk;
      size_t replan_start_index;
      {
        replan_start_index =
            std::min(path_index_ + static_cast<size_t>((replan_lookahead_sec_) /
                                                       constraints_.sampling_dt),
                     path_queue_.size());
        // ROS_INFO(
        //     "[Mav Local Planner][Plan Step] Current path index: %zu Replan start "
        //     "index: %zu",
        //     path_index_, replan_start_index);
        // Cut out the remaining snippet of the trajectory so we can do
        // something with it.
        std::copy(path_queue_.begin() + replan_start_index, path_queue_.end(),
                  std::back_inserter(path_chunk));
        if (path_chunk.size() == 0)
        {
          path_chunk.push_back(path_queue_.back());
          if (!nextWaypoint())
          {
            finishWaypoints();
          }
        }
      }
      mav_msgs::EigenTrajectoryPointVector path_replanned;
      bool replan_path = false;
      bool path_chunk_collision_free = isPathCollisionFree(path_chunk);

      if(verbose_){
          ROS_INFO(
          "[Mav Local Planner][Plan Step] Existing chunk is collision free? %d",
          path_chunk_collision_free);
      }

      updatePlannerStatus("Replanning", path_chunk_collision_free + 4);
      bool goal_reached = ((path_chunk.back().position_W - waypoint.position_W).norm() < kCloseEnough);
      if (path_chunk_collision_free)
      {
        if(verbose_){
            ROS_INFO(
            "[Mav Local Planner][Plan Step] Current plan is valid and still cannot get to the goal wp, just "
            "rollin' with it.");
        }

        nextWaypoint();
        return;
      }
      else
      {
        ROS_INFO("[Mav Local Planner][Plan Step] ABORTING! No local solution found.");
        // TODO(helenol): which order to abort in?
        abort();
      }
    }
    else
    {
      ROS_INFO("[Mav Local Planner][Plan Step] Trying to plan from scratch.");

      // There's nothing planned so far! So we plan from the current odometry.
      mav_msgs::EigenTrajectoryPoint current_point;
      current_point.position_W = odometry_.position_W;
      current_point.orientation_W_B = odometry_.orientation_W_B;

      // Check if the current waypoint is basically the odometry.
      if ((current_point.position_W - waypoint.position_W).norm() <
          kCloseEnough)
      {
        if (nextWaypoint())
        {
          waypoint = waypoints_[current_waypoint_];
        }
        else
        {
          return;
        }
      }

      success = loco_planner_.getTrajectoryTowardGoal(current_point, waypoint,
                                                      &trajectory);
      ROS_INFO("[Mav Local Planner][Plan Step] Planning success? %d", success);

      if (success)
      {
        num_aborts_ = 0;
        updatePlannerStatus("Planning_success", 3);
        path_found_ = true;
        if (trajectory.getMaxTime() <= 0.1)
        {
          nextWaypoint();
        }
        else
        {
          // Copy this straight into the queue.
          num_failures_ = 0;
          mav_msgs::EigenTrajectoryPointVector path;
          mav_trajectory_generation::sampleWholeTrajectory(
              trajectory, constraints_.sampling_dt, &path);
          replacePath(path);
        }
      }
      else
      {
        path_found_ = false;
        bool plan_b = dealWithFailure();
        if (plan_b)
        {

          if (waypoints_[current_waypoint_].position_W.z() < 1e-10)
          {
            ROS_INFO("[dealWithFailure] Z negative !!!");

            waypoints_[current_waypoint_].position_W.z() = waypoint.position_W.z();
          }

          bool success_b = loco_planner_.getTrajectoryTowardGoal(current_point, waypoints_[current_waypoint_], // waypoint
                                                                 &trajectory);
          ROS_INFO("[Mav Local Planner][Plan Step] Planning success? %d", success);

          if (success_b)
          {
            if (trajectory.getMaxTime() <= 0.1)
            {
              nextWaypoint();
              ROS_INFO("[Mav Local Planner][Not Replace Path]");
            }
            else
            {
              ROS_INFO("[Mav Local Planner][Replace Path]");
              // Copy this straight into the queue.
              num_failures_ = 0;
              mav_msgs::EigenTrajectoryPointVector path;
              mav_trajectory_generation::sampleWholeTrajectory(
                  trajectory, constraints_.sampling_dt, &path);
              replacePath(path);
              path_found_ = true;
            }
          }
        }
        // else
        // {
        //   // If dealWithFailure could not find an alternative wp, then try again.
        //   num_aborts_++;
        //   if (num_aborts_ < max_failures_)
        //   {
        //     avoidCollisionsTowardWaypoint();
        //   }
        // }
      }
    }
  }

  bool MavLocalPlanner::planPathThroughWaypoints(
      const mav_msgs::EigenTrajectoryPointVector &waypoints,
      mav_msgs::EigenTrajectoryPointVector *path)
  {
    CHECK_NOTNULL(path);
    bool success = false;
    if (smoother_name_ == "loco")
    {
      ROS_INFO("USING LOCO");
      if (waypoints.size() == 2)
      {
        ROS_INFO("USING getPathBetweenTwoPoints");
        success = loco_smoother_.getPathBetweenTwoPoints(waypoints[0],
                                                         waypoints[1], path);
      }
      else
      {
        ROS_INFO("USING getPathBetweenWaypoints");
        success = loco_smoother_.getPathBetweenWaypoints(waypoints, path);
      }
    }
    else if (smoother_name_ == "polynomial")
    {
      ROS_INFO("USING polynomial");
      success = poly_smoother_.getPathBetweenWaypoints(waypoints, path);
    }
    else if (smoother_name_ == "ramp")
    {
      ROS_INFO("USING ramp");
      success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
    }
    else
    {
      // Default case is ramp!
      ROS_ERROR(
          "[Mav Local Planner] Unknown smoother type %s, using ramp instead.",
          smoother_name_.c_str());
      success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
    }
    return success;
  }

  bool MavLocalPlanner::nextWaypoint()
  {
    if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size()) - 1)
    {
      current_waypoint_ = waypoints_.size() - 1;
      return false;
    }
    else
    {
      current_waypoint_++;
      return true;
    }
  }

  void MavLocalPlanner::finishWaypoints()
  {
    current_waypoint_ = waypoints_.size();
    updatePlannerStatus("Finished", 6);
    ROS_ERROR("%s[Mav Local Planner][Status] %s %s", COLOR4, "Finished", RESET_COLOR);
  }

  void MavLocalPlanner::replacePath(
      const mav_msgs::EigenTrajectoryPointVector &path)
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);
    path_queue_.clear();
    path_queue_ = path;
    path_queue_.front().orientation_W_B = odometry_.orientation_W_B;
    yaw_policy_.applyPolicyInPlace(&path_queue_);
    path_index_ = 0;
  }

  void MavLocalPlanner::startPublishingCommands()
  {
    // Call the service call to takeover publishing commands.
    if (position_hold_client_.exists())
    {
      std_srvs::Empty empty_call;
      position_hold_client_.call(empty_call);
    }
    // Publish the first set immediately, on this thread.
    commandPublishTimerCallback(ros::TimerEvent());

    // Need advanced timer options to assign callback queue to this timer.
    ros::TimerOptions timer_options(
        ros::Duration(command_publishing_dt_),
        boost::bind(&MavLocalPlanner::commandPublishTimerCallback, this, _1),
        &command_publishing_queue_);

    command_publishing_timer_ = nh_.createTimer(timer_options);
  }

  void MavLocalPlanner::commandPublishTimerCallback(
      const ros::TimerEvent &event)
  {
    constexpr size_t kQueueBuffer = 0;
    //planner_active = 0;
    if (path_index_ < path_queue_.size())
    {
      std::lock_guard<std::recursive_mutex> guard(path_mutex_);
      size_t number_to_publish = std::min<size_t>(
          std::floor(command_publishing_dt_ / constraints_.sampling_dt),
          path_queue_.size() - path_index_);

      size_t starting_index = 0;
      if (path_index_ != 0)
      {
        starting_index = path_index_ + kQueueBuffer;
        if (starting_index >= path_queue_.size())
        {
          starting_index = path_index_;
        }
      }

      size_t number_to_publish_with_buffer = std::min<size_t>(
          number_to_publish + mpc_prediction_horizon_ - kQueueBuffer,
          path_queue_.size() - starting_index);

      // TODO(helenol): do this without copy! Use iterators properly!
      mav_msgs::EigenTrajectoryPointVector::const_iterator first_sample =
          path_queue_.begin() + starting_index;
      mav_msgs::EigenTrajectoryPointVector::const_iterator last_sample =
          first_sample + number_to_publish_with_buffer;
      mav_msgs::EigenTrajectoryPointVector trajectory_to_publish(first_sample,
                                                                 last_sample);

      trajectory_msgs::MultiDOFJointTrajectory msg;
      msg.header.frame_id = local_frame_id_;
      msg.header.stamp = ros::Time::now();

      if(verbose_){
          ROS_INFO(
          "[Mav Local Planner][Command Publish] Publishing %zu samples of %zu. "
          "Start index: %zu Time: %f Start position: %f Start velocity: %f End "
          "time: %f End position: %f",
          trajectory_to_publish.size(), path_queue_.size(), starting_index,
          trajectory_to_publish.front().time_from_start_ns * 1.0e-9,
          trajectory_to_publish.front().position_W.x(),
          trajectory_to_publish.front().velocity_W.x(),
          trajectory_to_publish.back().time_from_start_ns * 1.0e-9,
          trajectory_to_publish.back().position_W.x());
      }

      mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_to_publish, &msg);

      command_pub_.publish(msg);
      path_index_ += number_to_publish;
      should_replan_.notify();
      planner_active = 1;
      planner_active_pub_.publish(planner_active);
    }else{
      planner_active = 0;
      planner_active_pub_.publish(planner_active);
    }
    // Does there need to be an else????
  }

  void MavLocalPlanner::abort()
  {
    updatePlannerStatus("Abort", 0);
    // No need to check anything on stop, just clear all the paths.
    clearTrajectory();
    // Make sure to clear the queue in the controller as well (we send about a
    // second of trajectories ahead).
    sendCurrentPose();
    startPublishingCommands();
  }

  void MavLocalPlanner::clearTrajectory()
  {
    if (path_queue_.size() <= path_index_)
    {
      ROS_WARN("Trying to pause an empty or finished trajectory queue!");
    }
    else
    {
      //command_publishing_spinner_.stop();
      command_publishing_timer_.stop();
    }

    std::lock_guard<std::recursive_mutex> guard(path_mutex_);
    path_queue_.clear();
    path_index_ = 0;
  }

  void MavLocalPlanner::sendCurrentPose()
  {
    // Sends the current pose with velocity 0 to the controller to clear the
    // controller's trajectory queue.
    // More or less an abort operation.
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    trajectory_msgs::MultiDOFJointTrajectory msg;
    msgMultiDofJointTrajectoryFromEigen(current_point, &msg);
    command_pub_.publish(msg);
  }

  bool MavLocalPlanner::startCallback(std_srvs::Empty::Request &request,
                                      std_srvs::Empty::Response &response)
  {
    if (path_queue_.size() <= path_index_)
    {
      ROS_WARN("Trying to start an empty or finished trajectory queue!");
      return false;
    }
    startPublishingCommands();
    return true;
  }

  bool MavLocalPlanner::pauseCallback(std_srvs::Empty::Request &request,
                                      std_srvs::Empty::Response &response)
  {
    if (path_queue_.size() <= path_index_)
    {
      ROS_WARN("Trying to pause an empty or finished trajectory queue!");
      return false;
    }
    command_publishing_timer_.stop();
    return true;
  }

  bool MavLocalPlanner::stopCallback(std_srvs::Empty::Request &request,
                                     std_srvs::Empty::Response &response)
  {
    abort();
    return true;
  }

  void MavLocalPlanner::visualizePath()
  {
    // TODO: Split trajectory into two chunks: before and after.
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker path_marker;
    {
      std::lock_guard<std::recursive_mutex> guard(path_mutex_);

      path_marker = createMarkerForPath(path_queue_, local_frame_id_,
                                        mav_visualization::Color::Black(),
                                        "local_path", 0.05);
    }
    marker_array.markers.push_back(path_marker);
    path_marker_pub_.publish(marker_array);
  }

  void MavLocalPlanner::visualizeRobotRadius(mav_msgs::EigenTrajectoryPoint &point)
  {
    // TODO: Split trajectory into two chunks: before and after.
    visualization_msgs::Marker robot_radius_marker;
    robot_radius_marker = createMarkerForRobotRadius(point, local_frame_id_,
                                                     mav_visualization::Color::Blue(),
                                                     "robot_radius", constraints_.robot_radius);
    robot_radius_marker_pub_.publish(robot_radius_marker);
  }

  double MavLocalPlanner::getMapDistance(const Eigen::Vector3d &position) const
  {
    double distance = 0.0;
    const bool kInterpolate = false;
    if (!esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
            position, kInterpolate, &distance))
    {
      return 0.0;
    }
    return distance;
  }

  double MavLocalPlanner::getMapDistanceAndGradient(
      const Eigen::Vector3d &position, Eigen::Vector3d *gradient) const
  {
    double distance = 0.0;
    const bool kInterpolate = false;
    if (!esdf_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(
            position, kInterpolate, &distance, gradient))
    {
      return 0.0;
    }
    return distance;
  }

  bool MavLocalPlanner::isPathCollisionFree(
      const mav_msgs::EigenTrajectoryPointVector &path) const
  {
    for (const mav_msgs::EigenTrajectoryPoint &point : path)
    {
      if (getMapDistance(point.position_W) < constraints_.robot_radius - 0.1)
      {
        return false;
      }
    }
    return true;
  }

  bool MavLocalPlanner::isPathFeasible(
      const mav_msgs::EigenTrajectoryPointVector &path) const
  {
    // This is easier to check in the trajectory but then we are limited in how
    // we do the smoothing.
    for (const mav_msgs::EigenTrajectoryPoint &point : path)
    {
      if (point.acceleration_W.norm() > constraints_.a_max + 1e-2)
      {
        return false;
      }
      if (point.velocity_W.norm() > constraints_.v_max + 1e-2)
      {
        return false;
      }
    }
    return true;
  }

  bool MavLocalPlanner::dealWithFailure()
  {

    updatePlannerStatus("DealWithFailure", 1);

    if (current_waypoint_ < 0)
    {
      ROS_ERROR("current wp < 0");
      return false;
    }

    constexpr double kCloseEnough = 0.05;
    mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
    mav_msgs::EigenTrajectoryPoint goal = waypoint;
    if (temporary_goal_ &&
        static_cast<int64_t>(waypoints_.size()) > current_waypoint_ + 1)
    {
      goal = waypoints_[current_waypoint_ + 1];
    }
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    mav_msgs::EigenTrajectoryPoint current_goal;
    if (!goal_selector_.selectNextGoal(goal, waypoint, current_point,
                                       &current_goal))
    {
      num_failures_++;
      if (num_failures_ > max_failures_)
      {
        current_waypoint_ = -1;
      }
      return false;
    }
    else
    {
      ROS_INFO("Select Next Goal succeded!!!");
      if ((current_goal.position_W - waypoint.position_W).norm() < kCloseEnough)
      {
        // Goal is unchanged. :(
        temporary_goal_ = false;
        ROS_INFO("Goal unchanged!!!, dist %f, kcloseenough %f", (current_goal.position_W - waypoint.position_W).norm(), kCloseEnough);
        return false;
      }
      else if ((current_goal.position_W - goal.position_W).norm() <
               kCloseEnough)
      {
        // This is just the next waypoint that we're trying to go to.
        current_waypoint_++;
        temporary_goal_ = false;
        ROS_INFO("Next wp !!!, goal pos  %f, %f, %f", goal.position_W.x(), goal.position_W.y(), goal.position_W.z());
        return true;
      }
      else
      {
        // Then this is something different!
        temporary_goal_ = true;
        waypoints_.insert(waypoints_.begin() + current_waypoint_, current_goal);
        ROS_INFO("Temporary goal is %f, %f, %f", current_goal.position_W.x(), current_goal.position_W.y(), current_goal.position_W.z());
        return true;
      }
    }
  }

  void MavLocalPlanner::updatePlannerStatus(const char *status, std::int32_t value)
  {
    static const char *const colors[4] = {COLOR1, COLOR5, COLOR3, COLOR2};
    if (value < 4)
    {
      if (value < 0)
      {
        ROS_ERROR("%s[Mav Local Planner][Status] %s %s", COLOR7, status, RESET_COLOR);
      }
      else
      {
        ROS_ERROR("%s[Mav Local Planner][Status] %s %s", colors[value], status, RESET_COLOR);
      }
    }

    planner_status_pub_.publish(value);
  }

} // namespace mav_planning
