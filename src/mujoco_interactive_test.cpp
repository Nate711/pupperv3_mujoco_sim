#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "actuator_model.hpp"
#include "mujoco_interactive_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  for (int i = 0; i < argc; i++) {
    std::cout << argv[i] << " ";
  }

  // Construct actuator models
  ActuatorParams params(
      /*kp=*/7.5,  // value used before command recvd. 7.5 is good for trotting
      /*kd=*/0.5,  // value used before command recvd. 0.5 is good for trotting
      /*bus_voltage =*/30.0,
      /*kt=*/0.1,
      /*phase_resistance=*/0.7,
      /*saturation_torque=*/4.5,
      /*software_torque_limit =*/3.0);
  PIDActuatorModel actuator_model(params);
  std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models(
      12, std::make_shared<PIDActuatorModel>(params));

  std::vector<std::string> joint_names;
  const std::string joint_names_[12] = {
      "leg_front_r_1", "leg_front_r_2", "leg_front_r_3", "leg_front_l_1",
      "leg_front_l_2", "leg_front_l_3", "leg_back_r_1",  "leg_back_r_2",
      "leg_back_r_3",  "leg_back_l_1",  "leg_back_l_2",  "leg_back_l_3",
  };
  for (int i = 0; i < 12; i++) {
    joint_names.push_back(joint_names_[i]);
  }

  std::shared_ptr<MujocoInteractiveNode> node_ptr;
  node_ptr = std::make_shared<MujocoInteractiveNode>(joint_names, actuator_models);

  // Start mujoco physics simulation thread
  node_ptr->start_simulation();

  // Start calibration thread
  node_ptr->calibrate_motors_detached();

  // Start GUI thread
  node_ptr->run_gui_detached();

  // Doesn't work because method is static and therefore this instantiates a new version of
  // mujoco_core_interactive.hpp
  // std::thread run_gui([]() { mujoco_interactive::run_gui_blocking(); });
  // mujoco_interactive::run_gui_detached();

  // Start spinning node callbacks
  rclcpp::spin(node_ptr);
  std::cout << "---------CALLING RCLCPP SHUTDOWN----------" << std::endl;
  rclcpp::shutdown();
  return 0;
}
