#include "fpv_aggressive_trajectories/fpv_aggressive_trajectories.h"

#include <autopilot/autopilot_states.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_common/trajectory_point.h>
#include <trajectory_generation_helper/circle_trajectory_helper.h>
#include <trajectory_generation_helper/flip_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>

namespace fpv_aggressive_trajectories {

FPVAggressiveTrajectories::FPVAggressiveTrajectories() {
  if (!loadParameters()) {
    ROS_ERROR("[%s] Failed to load all parameters",
              ros::this_node::getName().c_str());
    ros::shutdown();
  }

  toggle_experiment_sub_ = nh_.subscribe(
      "fpv_quad_looping/execute_trajectory", 1,
      &FPVAggressiveTrajectories::startExecutionCallback, this);

//  if (!computeSplitS()) {
  if (!computeLoop()) {
//  if (!computeSpin()) {
    ROS_ERROR("[%s] Failed to compute feasible trajectories",
              ros::this_node::getName().c_str());
    ros::shutdown();
  }
}

FPVAggressiveTrajectories::~FPVAggressiveTrajectories() {
}

bool FPVAggressiveTrajectories::computeLoop() {
  const Eigen::Vector3d circle_center = Eigen::Vector3d(1.5, 0.0, 1.4);
  const double circle_radius = 0.75;
  const double figure_z_rotation_angle = 0.785398163;

  const double max_thrust = 9.81 + 1.5 * pow(circle_velocity_, 2.0) / circle_radius;
  const double max_roll_pitch_rate = 3.0;

  const Eigen::Quaterniond q_W_P = Eigen::Quaterniond(
      Eigen::AngleAxisd(figure_z_rotation_angle, Eigen::Vector3d::UnitZ()));
  desired_heading_ = quadrotor_common::wrapMinusPiToPi(
      desired_heading_ + figure_z_rotation_angle);

  // Compute Circle trajectory
  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          circle_center, figure_z_rotation_angle, circle_radius,
          circle_velocity_, M_PI / 2.0, -3.0 / 2.0 * M_PI, exec_loop_rate_);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_, &circle_trajectory);

  quadrotor_common::TrajectoryPoint circle_enter_state =
      circle_trajectory.points.front();

  // Start position relative to circle center
  const Eigen::Vector3d start_pos_P = Eigen::Vector3d(-2.0, 0.0, 0.5);
  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = q_W_P * start_pos_P + circle_center;

  quadrotor_common::Trajectory enter_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, circle_enter_state, 4, 1.1 * circle_velocity_,
          max_thrust, max_roll_pitch_rate, exec_loop_rate_);
  trajectory_generation_helper::heading::addConstantHeading(desired_heading_,
                                                            &enter_trajectory);

  // End position relative to circle center
  const Eigen::Vector3d end_pos_P = Eigen::Vector3d(2.3, 0.0, 0.5); // nice breaking forward flip
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = q_W_P * end_pos_P + circle_center;

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          circle_enter_state, end_state, 4, 1.1 * circle_velocity_, 10.0 * max_thrust,
          10.0 * max_roll_pitch_rate, exec_loop_rate_);
  trajectory_generation_helper::heading::addConstantHeading(desired_heading_,
                                                            &exit_trajectory);

  trajectory_list_.push_back(enter_trajectory);
  trajectory_list_.push_back(circle_trajectory);
  trajectory_list_.push_back(exit_trajectory);

  return !(enter_trajectory.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED
      || circle_trajectory.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED
      || exit_trajectory.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool FPVAggressiveTrajectories::computeSpin() {
  // compute debug flip
  quadrotor_common::TrajectoryPoint debug_start_state;
  debug_start_state.position = Eigen::Vector3d(0.0, 0.0, 2.0);
  double revolution_angle = 2.0 * M_PI;
  const double max_thrust = 20.0;
  const double max_roll_pitch_rate = 30.0;

  double spin_time = 0.5;
  double init_accel_time = 0.5;
  double init_lin_acc = 20.0;
  double coast_acc = 2.5; // thrust applied during the spin maneuver
  double final_hover_time = 2.0;
  quadrotor_common::Trajectory
      debug_spin_trajectory = trajectory_generation_helper::flips::computeFlipTrajectory(debug_start_state,
                                                                                         init_accel_time,
                                                                                         init_lin_acc,
                                                                                         revolution_angle,
                                                                                         spin_time,
                                                                                         coast_acc,
                                                                                         final_hover_time,
                                                                                         exec_loop_rate_);
  // exit trajectory is needed to keep autopilot from crashing...
  const Eigen::Vector3d end_pos_P = Eigen::Vector3d(2.3, 0.0, 0.5);
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = end_pos_P;

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          debug_spin_trajectory.points.back(), end_state, 4, 1.1 * circle_velocity_, max_thrust,
          max_roll_pitch_rate, exec_loop_rate_);
  trajectory_generation_helper::heading::addConstantHeading(desired_heading_,
                                                            &exit_trajectory);

  trajectory_list_.push_back(debug_spin_trajectory);
  trajectory_list_.push_back(exit_trajectory);

  return !(debug_spin_trajectory.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED
      || exit_trajectory.trajectory_type
          == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool FPVAggressiveTrajectories::computeSplitS() {

  const double circle_radius = 0.75;
  const double figure_z_rotation_angle = 0.0;
  const double max_thrust = 30.0;
  const double max_roll_pitch_rate = 30.0;


  // enter trajectory
  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = Eigen::Vector3d(0.0, 0.0, 2.0);
  quadrotor_common::TrajectoryPoint circle_enter_state;
  circle_enter_state.position = Eigen::Vector3d(4.0, 0.0, 2.0);
  circle_enter_state.velocity = Eigen::Vector3d(circle_velocity_, 0.0, 0.0);
  quadrotor_common::Trajectory enter_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, circle_enter_state, 4, 1.1 * circle_velocity_,
          max_thrust, max_roll_pitch_rate, exec_loop_rate_);
  trajectory_generation_helper::heading::addConstantHeading(desired_heading_,
                                                            &enter_trajectory);

  // half-circle trajectory
  double start_angle = M_PI / 2.0;
  double end_angle = -2.0 / 4.0 * M_PI;
  const Eigen::Vector3d circle_center = Eigen::Vector3d(4.0, 0.0, 2.0 + circle_radius);
  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          circle_center, figure_z_rotation_angle, circle_radius,
          circle_velocity_, start_angle, end_angle, exec_loop_rate_);

  // spin trajectory
  quadrotor_common::TrajectoryPoint circle_exit_state =
      circle_trajectory.points.back();
  double spin_time = 0.3;
  double init_accel_time = 0.0;
  double init_lin_acc = 20.0;
  double spin_angle = M_PI;
  double coast_acc = 2.5; // thrust applied during the spin maneuver
  double final_hover_time = 0.0;
  quadrotor_common::Trajectory
      spin_trajectory = trajectory_generation_helper::flips::computeFlipTrajectory(circle_exit_state,
                                                                                   init_accel_time,
                                                                                   init_lin_acc,
                                                                                   spin_angle,
                                                                                   spin_time,
                                                                                   coast_acc,
                                                                                   final_hover_time,
                                                                                   exec_loop_rate_);

  // exit trajectory
//  quadrotor_common::TrajectoryPoint spin_exit_state = spin_trajectory.points.back();
// hack: to make the exit trajectory feasible, set some of the start states to zero
  quadrotor_common::TrajectoryPoint spin_exit_state;
  spin_exit_state.position = spin_trajectory.points.back().position;
  spin_exit_state.velocity = spin_trajectory.points.back().velocity;
  spin_exit_state.orientation = spin_trajectory.points.back().orientation;

  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = Eigen::Vector3d(0.0, 0.0, 4.0);
  end_state.orientation = quadrotor_common::eulerAnglesZYXToQuaternion(Eigen::Vector3d(0.0, 0.0, M_PI));

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          spin_exit_state, end_state, 4, spin_exit_state.velocity.norm(),
          max_thrust, max_roll_pitch_rate, exec_loop_rate_);
  trajectory_generation_helper::heading::addConstantHeading(M_PI,
                                                            &exit_trajectory);

  // Debug output
  std::cout << static_cast<int>(enter_trajectory.trajectory_type) << std::endl;
  std::cout << static_cast<int>(circle_trajectory.trajectory_type) << std::endl;
  std::cout << static_cast<int>(spin_trajectory.trajectory_type) << std::endl;
  std::cout << static_cast<int>(exit_trajectory.trajectory_type) << std::endl;

  trajectory_list_.push_back(enter_trajectory);
  trajectory_list_.push_back(circle_trajectory);
  trajectory_list_.push_back(spin_trajectory);
  trajectory_list_.push_back(exit_trajectory);

  return !(enter_trajectory.trajectory_type
      == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED
      || circle_trajectory.trajectory_type
          == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED
      || spin_trajectory.trajectory_type
          == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED
      || exit_trajectory.trajectory_type
          == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

void FPVAggressiveTrajectories::startExecutionCallback(
    const std_msgs::Empty::ConstPtr &msg) {
  if (autopilot_helper_.getCurrentAutopilotState() != autopilot::States::HOVER) {
    return;
  }

  // Go to start position
  autopilot_helper_.sendPoseCommand(trajectory_list_.front().points.front().position,
                                    trajectory_list_.front().points.front().heading);

  ros::Duration(0.5).sleep();

  // Wait for quad to reach start position
  autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER,
                                                  20.0, exec_loop_rate_);

  printf("----------------------------------------\n");
  printf("Start new trajectory.\n");
  printf("----------------------------------------\n");
  for (const quadrotor_common::Trajectory &trajectory : trajectory_list_) {
    autopilot_helper_.sendTrajectory(trajectory);
    ros::Duration(0.2).sleep();
  }
}

bool FPVAggressiveTrajectories::loadParameters() {
  if (!quadrotor_common::getParam("desired_yaw_P", desired_heading_, 0.0))
    return false;

  if (!quadrotor_common::getParam("loop_rate", exec_loop_rate_, 55.0))
    return false;

  if (!quadrotor_common::getParam("circle_velocity", circle_velocity_, 1.0))
    return false;

  if (!quadrotor_common::getParam("n_loops", n_loops_, 1))
    return false;

  return true;
}

} // namespace fpv_aggressive_trajectories

int main(int argc, char **argv) {
  ros::init(argc, argv, "fpv_aggressive_trajectories");
  fpv_aggressive_trajectories::FPVAggressiveTrajectories fpv_aggressive_trajectories;

  ros::spin();

  return 0;
}
