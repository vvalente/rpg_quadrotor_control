#include "simple_controller/simple_controller.h"

#include <quadrotor_common/math_common.h>

namespace simple_controller {

SimpleController::SimpleController() {
}

SimpleController::~SimpleController() {
}

#define CONSTRAIN(X, MIN, MAX) ((X) < (MIN) ? (MIN) : ((X) > (MAX) ? (MAX) : (X)))

quadrotor_common::ControlCommand SimpleController::off() {
  quadrotor_common::ControlCommand command;

  command.zero();

  return command;
}

quadrotor_common::ControlCommand SimpleController::run(
    const quadrotor_common::QuadStateEstimate &state_estimate,
    const quadrotor_common::Trajectory &reference_trajectory,
    const SimpleControllerParams &config) {

  quadrotor_common::ControlCommand command;
  command.armed = true;

  quadrotor_common::TrajectoryPoint reference_state(
      reference_trajectory.points.front());


  /*************************
     * controller parameters *
     *************************/
  // TODO: remove local variable copies
  // position controller
  double tauPosXY = config.tauPosXY;
  double zetaPosXY = config.zetaPosXY;

  double tauPosZ = config.tauPosZ;
  double zetaPosZ = config.zetaPosZ;

  // attitude controller
  double tauAttXY = config.tauAttXY;
  double tauAttZ = config.tauAttZ;

  // constraints
  double minThrust = config.minThrust;
  double maxThrust = config.maxThrust;
  double maxOmegaXY = config.maxOmegaXY;
  double maxOmegaZ = config.maxOmegaZ;

  /***********************
   * position controller *
   ***********************/
  Eigen::Vector3d posErr = reference_state.position - state_estimate.position;
  Eigen::Vector3d velErr = reference_state.velocity - state_estimate.velocity;

  Eigen::Vector3d accCmd(0.0, 0.0, 9.81);

  double gainPXY = 1.0 / (tauPosXY * tauPosXY);
  double gainPZ = 1.0 / (tauPosZ * tauPosZ);

  accCmd.x() = accCmd.x() + gainPXY * posErr.x();
  accCmd.y() = accCmd.y() + gainPXY * posErr.y();
  accCmd.z() = accCmd.z() + gainPZ * posErr.z();

  double gainDXY = 2.0 * zetaPosXY / tauPosXY;
  double gainDZ = 2.0 * zetaPosZ / tauPosZ;

  accCmd.x() = accCmd.x() + gainDXY * velErr.x();
  accCmd.y() = accCmd.y() + gainDXY * velErr.y();
  accCmd.z() = accCmd.z() + gainDZ * velErr.z();

  accCmd += reference_state.acceleration;

  // compute thrust command
  double thrustCmd = accCmd.z() / state_estimate.orientation.toRotationMatrix()(2, 2);

  /***********************
   * attitude controller *
   ***********************/
  Eigen::Vector3d I_eZ_I(0.0, 0.0, 1.0);
  Eigen::Quaterniond quatDes = Eigen::Quaterniond::FromTwoVectors(I_eZ_I, accCmd)
      * Eigen::Quaterniond(std::cos(0.5 * reference_state.heading),
                           0.0,
                           0.0,
                           std::sin(0.5 * reference_state.heading));
  Eigen::Quaterniond quatErr = state_estimate.orientation.inverse() * quatDes;

  printf("error quaternion x y z w: %.2f\t%.2f\t%.2f\t%.2f\n",
         quatErr.x(),
         quatErr.y(),
         quatErr.z(),
         quatErr.w());

  // compute reduced attitude and yaw error
  double q0q3sqrt = std::sqrt(quatErr.w() * quatErr.w() + quatErr.z() * quatErr.z());
  Eigen::Quaterniond quatErrRed, quatErrYaw;
  if (q0q3sqrt < 1e-6) {
//      double omegaXYSqrt = std::sqrt(state.omega.x()*state.omega.x() + state.omega.y()*state.omega.y());
    double omegaXYSqrt = std::sqrt(state_estimate.bodyrates.x() * state_estimate.bodyrates.x()
                                       + state_estimate.bodyrates.y() * state_estimate.bodyrates.y());
    if (omegaXYSqrt < 1e-1) {
      quatErrRed = quatErr;
    } else {
      quatErrRed.w() = 0.0;
      quatErrRed.x() = state_estimate.bodyrates.x();
      quatErrRed.y() = state_estimate.bodyrates.y();
      quatErrRed.z() = 0.0;
      quatErrRed.normalize();
    }

    quatErrYaw = quatErrRed.inverse() * quatErr;
  } else {
    quatErrRed = Eigen::Quaterniond(quatErr.w() * quatErr.w() + quatErr.z() * quatErr.z(),
                                    quatErr.w() * quatErr.x() - quatErr.y() * quatErr.z(),
                                    quatErr.w() * quatErr.y() + quatErr.x() * quatErr.z(),
                                    0.0).coeffs() * 1.0 / q0q3sqrt;

    quatErrYaw = Eigen::Quaterniond(quatErr.w(),
                                    0.0,
                                    0.0,
                                    quatErr.z()).coeffs() * 1.0 / q0q3sqrt;
  }

  // ensure error is smaller than pi
  if (quatErr.w() < 0.0) {
    quatErrYaw.coeffs() *= -1.0;
  }

  // nonliner attitude control law
  Eigen::Vector3d omegaDesTilde = reference_state.bodyrates;
  // NOTE: The above is not entirely correct! You might want to use "state_estimate_.orientation.inverse()*reference_state.orientation"
  // instead of "quatErr".
  Eigen::Vector3d omegaCmd;
  omegaCmd.x() = 2.0 / tauAttXY * quatErrRed.x() + omegaDesTilde.x();
  omegaCmd.y() = 2.0 / tauAttXY * quatErrRed.y() + omegaDesTilde.y();
  omegaCmd.z() = 2.0 / tauAttZ * quatErrYaw.z() + omegaDesTilde.z();

  /*********************
   * end of controller *
   *********************/

  command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
  command.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // constrain commands
  command.collective_thrust = CONSTRAIN(thrustCmd, minThrust, maxThrust);
  command.bodyrates.x() = CONSTRAIN(omegaCmd.x(), -maxOmegaXY, maxOmegaXY);
  command.bodyrates.y() = CONSTRAIN(omegaCmd.y(), -maxOmegaXY, maxOmegaXY);
  command.bodyrates.z() = CONSTRAIN(omegaCmd.z(), -maxOmegaZ, maxOmegaZ);

//    command.collective_thrust = CONSTRAIN((reference_state.acceleration + Eigen::Vector3d(0.0, 0.0, 9.81)).norm() , minThrust, maxThrust);
//    command.bodyrates.x() = CONSTRAIN(reference_state.bodyrates.x(), -maxOmegaXY, maxOmegaXY);
//    command.bodyrates.y() = CONSTRAIN(reference_state.bodyrates.y(), -maxOmegaXY, maxOmegaXY);
//    command.bodyrates.z() = CONSTRAIN(reference_state.bodyrates.z(), -maxOmegaZ, maxOmegaZ);

//  printf("command thrust brx bry brz: %.2f\t%.2f\t%.2f\t%.2f\n",
//         command.collective_thrust,
//         command.bodyrates.x(),
//         command.bodyrates.y(),
//         command.bodyrates.z());
//  printf("================================================================\n");

  command.angular_accelerations = Eigen::Vector3d(0.0, 0.0, 0.0);
  command.timestamp = ros::Time::now();
  command.expected_execution_time = command.timestamp + ros::Duration(0.02);

  return command;
}

} // namespace simple_controller

