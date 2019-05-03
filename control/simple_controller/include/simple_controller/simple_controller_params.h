#pragma once

#include <quadrotor_common/parameter_helper.h>

namespace simple_controller {

class SimpleControllerParams {
 public:
  SimpleControllerParams() : tauPosXY(0.0), zetaPosXY(0.0), tauPosZ(0.0), zetaPosZ(0.0),
                             tauAttXY(0.0), tauAttZ(0.0), minThrust(0.0), maxThrust(0.0),
                             maxOmegaXY(0.0), maxOmegaZ(0.0) {
  }

  ~SimpleControllerParams() {
  }

  bool loadParameters(const ros::NodeHandle &pnh) {
    const std::string path_rel_to_node = "simple_controller";

    if (!quadrotor_common::getParam(path_rel_to_node + "/tauPosXY", tauPosXY, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/zetaPosXY", zetaPosXY, pnh)) {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/tauPosZ", tauPosZ, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/zetaPosZ", zetaPosZ, pnh)) {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/tauAttXY", tauAttXY, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/tauAttZ", tauAttZ, pnh)) {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/minThrust",
                                    minThrust, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/maxThrust",
                                    maxThrust, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/maxOmegaXY",
                                    maxOmegaXY, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/maxOmegaZ",
                                    maxOmegaZ, pnh)) {
      return false;
    }

    return true;
  }

  // position controller
  double tauPosXY;
  double zetaPosXY;

  double tauPosZ;
  double zetaPosZ;

  // attitude controller
  double tauAttXY;
  double tauAttZ;

  // constraints
  double minThrust;
  double maxThrust;
  double maxOmegaXY;
  double maxOmegaZ;
};

} // namespace simple_controller
