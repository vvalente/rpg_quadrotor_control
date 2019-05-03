#pragma once

#include <Eigen/Dense>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_common/trajectory.h>
#include <ros/ros.h>

#include "simple_controller/simple_controller_params.h"

namespace simple_controller
{

class SimpleController
{
public:
  SimpleController();
  ~SimpleController();

  quadrotor_common::ControlCommand off();
  quadrotor_common::ControlCommand run(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const quadrotor_common::Trajectory& reference_trajectory,
      const SimpleControllerParams& config);

};

} // namespace simple_controller
