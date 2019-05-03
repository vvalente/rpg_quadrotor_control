#include "autopilot/autopilot.h"
#include "position_controller/position_controller.h"
#include "position_controller/position_controller_params.h"
#include <simple_controller/simple_controller.h>
#include <simple_controller/simple_controller_params.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "autopilot");

  //choose controller to instantiate
  ros::NodeHandle pnh = ros::NodeHandle("~");
  std::string desired_controller;
  pnh.param<std::string>("standard_controller", desired_controller, "none");

  if (desired_controller == "position_controller") {
    autopilot::AutoPilot<position_controller::PositionController,
                         position_controller::PositionControllerParams> autopilot;
    std::cout << "Instantiated position_controller" << std::endl;

    ros::spin();
  }
  if (desired_controller == "simple_controller") {
    autopilot::AutoPilot<simple_controller::SimpleController,
                         simple_controller::SimpleControllerParams> autopilot;
    std::cout << "Instantiated simple_controller" << std::endl;

    ros::spin();
  }

  std::cout << "invalid controller selected" << std::endl;

  return 0;
}
