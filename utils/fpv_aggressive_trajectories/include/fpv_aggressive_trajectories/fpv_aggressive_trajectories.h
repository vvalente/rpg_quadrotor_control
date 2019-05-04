#pragma once

#include <autopilot/autopilot_helper.h>
#include <Eigen/Core>
#include <quadrotor_common/trajectory.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace fpv_aggressive_trajectories
{

class FPVAggressiveTrajectories
{
public:
  FPVAggressiveTrajectories();
  virtual ~FPVAggressiveTrajectories();

private:
  ros::NodeHandle nh_;

  ros::Subscriber toggle_experiment_sub_;

  bool computeLoop();
  bool computeSpin();
  bool computeSplitS();
  void startExecutionCallback(const std_msgs::Empty::ConstPtr& msg);

  bool loadParameters();

  autopilot_helper::AutoPilotHelper autopilot_helper_;

  std::list<quadrotor_common::Trajectory> trajectory_list_;

  // Parameters
  double desired_heading_;
  double exec_loop_rate_;
  double circle_velocity_;
  int n_loops_;
};

} // namespace fpv_aggressive_trajectories
