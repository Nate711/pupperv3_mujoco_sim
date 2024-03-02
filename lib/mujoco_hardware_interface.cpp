#include "mujoco_hardware_interface.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "actuator_model.hpp"
#include "mujoco_core_interactive.hpp"

namespace mujoco_hardware_interface {
MujocoHardwareInterface::~MujocoHardwareInterface() {
  // Deactivate everything when ctrl-c is pressed
  on_deactivate(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_state_positions_.resize(info_.joints.size(), 0.0);
  hw_state_velocities_.resize(info_.joints.size(), 0.0);

  hw_command_positions_.resize(info_.joints.size(), 0.0);
  hw_command_velocities_.resize(info_.joints.size(), 0.0);
  hw_command_efforts_.resize(info_.joints.size(), 0.0);
  hw_command_kps_.resize(info_.joints.size(), 0.0);
  hw_command_kds_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // Set limits for each joint
    hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
    hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
    hw_actuator_velocity_maxs_.push_back(std::stod(joint.parameters.at("velocity_max")));
    hw_actuator_effort_maxs_.push_back(std::stod(joint.parameters.at("effort_max")));
    hw_actuator_kp_maxs_.push_back(std::stod(joint.parameters.at("kp_max")));
    hw_actuator_kd_maxs_.push_back(std::stod(joint.parameters.at("kd_max")));

    // Homing parameters
    hw_actuator_homing_stages_.push_back(std::stoi(joint.parameters.at("homing_stage")));
    hw_actuator_homing_velocities_.push_back(std::stod(joint.parameters.at("homing_velocity")));
    hw_actuator_homing_kps_.push_back(std::stod(joint.parameters.at("homing_kp")));
    hw_actuator_homing_kds_.push_back(std::stod(joint.parameters.at("homing_kd")));
    hw_actuator_homed_positions_.push_back(std::stod(joint.parameters.at("homed_position")));
    hw_actuator_zero_positions_.push_back(0.0);
    hw_actuator_homing_torque_thresholds_.push_back(
        std::stod(joint.parameters.at("homing_torque_threshold")));
    hw_actuator_is_homed_.push_back(false);
  }

  imu_roll_ = std::stod(info_.sensors[0].parameters.at("roll"));
  imu_pitch_ = std::stod(info_.sensors[0].parameters.at("pitch"));
  imu_yaw_ = std::stod(info_.sensors[0].parameters.at("yaw"));

  // TODO(nathan-kau): add support for mujoco imus
  //   // Set up the IMU
  //   imu_ = new BNO055(IMU_I2C_DEVICE_NUMBER);

  // Set up mujoco simulation
  mujoco_interactive::init();
  // TODO filename should be passed as parameter
  std::string model_xml =
      "/home/parallels/pupperv3_ws/src/pupper_mujoco_sim/src/urdf/pupper_v3_2_floating_base.xml";
  // std::string model_xml =
  //     "/home/parallels/pupperv3_ws/src/pupper_mujoco_sim/src/urdf/pupper_v3_2_fixed_base.xml";
  float timestep = 1e-4;
  // Construct actuator models
  // TODO get rid of kp and kd
  // TODO set from xml file
  ActuatorParams params(
      /*kp=*/7.5,  // value used before command recvd. 7.5 is good for trotting
      /*kd=*/0.5,  // value used before command recvd. 0.5 is good for trotting
      /*bus_voltage =*/24.0,
      /*kt=*/0.04,
      /*phase_resistance=*/0.7,
      /*saturation_torque=*/4.5,
      /*software_torque_limit =*/3.0);
  PIDActuatorModel actuator_model(params);
  std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models(
      info_.joints.size(), std::make_shared<PIDActuatorModel>(params));

  mju::strcpy_arr(mujoco_interactive::filename, model_xml.c_str());
  mujoco_interactive::settings.loadrequest = 1;
  mujoco_interactive::loadmodel();
  mujoco_interactive::set_timestep(timestep);
  mujoco_interactive::set_actuator_models(actuator_models);

  // Necessary if you want to run rendering in a different thread than
  // the one this Node was created in
  mujoco_interactive::detach_opengl_context_from_thread();

  // Start physics simulation
  mujoco_interactive::start_simulation();

  // Start GUI thread
  mujoco_interactive::run_gui_detached();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add joint state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
  }

  // Add IMU state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x",
                                                                   &hw_state_imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y",
                                                                   &hw_state_imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z",
                                                                   &hw_state_imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w",
                                                                   &hw_state_imu_orientation_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

  return state_interfaces;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_state_positions_.size(); i++) {
    hw_state_positions_[i] = 0.0;
    hw_state_velocities_[i] = 0.0;
    hw_command_positions_[i] = 0.0;
    hw_command_velocities_[i] = 0.0;
    hw_command_efforts_[i] = 0.0;
    hw_command_kps_[i] = 7.5;  // 0.0;
    hw_command_kds_[i] = 0.5;  // 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
MujocoHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kp", &hw_command_kps_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kd", &hw_command_kds_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Start calibration thread
  mujoco_interactive::calibrate_motors_detached();
  // You can comment out calibration and override calibration to be true if you want
  // mujoco_interactive::is_robot_calibrated_ = true;
  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // TODO disable actuators somehow
  hw_command_kps_.resize(info_.joints.size(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoHardwareInterface::read(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration &period) {
  hw_state_positions_ = mujoco_interactive::actuator_positions();
  hw_state_velocities_ = mujoco_interactive::actuator_velocities();

  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"),
              "READ. state_pos: %f %f %f | %f %f %f | %f %f %f | %f %f %f",
              hw_state_positions_.at(0), hw_state_positions_.at(1), hw_state_positions_.at(2),
              hw_state_positions_.at(3), hw_state_positions_.at(4), hw_state_positions_.at(5),
              hw_state_positions_.at(6), hw_state_positions_.at(7), hw_state_positions_.at(8),
              hw_state_positions_.at(9), hw_state_positions_.at(10), hw_state_positions_.at(11));

  // Read the IMU
  // TODO get from mujoco

  tf2::Quaternion imu_quat, offset_quat, corrected_quat;
  tf2::Matrix3x3 rotation_matrix;

  // Getting the original IMU quaternion
  // TODO
  // imu_quat.setValue(imu_output_.quat.x(), imu_output_.quat.y(), imu_output_.quat.z(),
  //                   imu_output_.quat.w());

  // Setting the offset quaternion based on your YAW, PITCH, ROLL offsets
  offset_quat.setRPY(imu_roll_, imu_pitch_, imu_yaw_);
  offset_quat = offset_quat.inverse();

  // Applying the offset to the IMU quaternion
  corrected_quat = imu_quat * offset_quat;
  corrected_quat.normalize();  // Normalizing the quaternion to ensure it's a valid rotation

  rotation_matrix.setRotation(offset_quat);

  // Rotating the angular velocity
  // TODO
  tf2::Vector3 angular_velocity;
  // tf2::Vector3 angular_velocity(imu_output_.gyro.x(), imu_output_.gyro.y(),
  // imu_output_.gyro.z()); angular_velocity = rotation_matrix * angular_velocity;

  // Rotating the linear acceleration
  // TODO
  tf2::Vector3
      linear_acceleration;  //(imu_output_.acc.x(), imu_output_.acc.y(), imu_output_.acc.z());
  linear_acceleration = rotation_matrix * linear_acceleration;

  // Updating the state interfaces with corrected values
  hw_state_imu_orientation_[0] = corrected_quat.x();
  hw_state_imu_orientation_[1] = corrected_quat.y();
  hw_state_imu_orientation_[2] = corrected_quat.z();
  hw_state_imu_orientation_[3] = corrected_quat.w();

  hw_state_imu_angular_velocity_[0] = angular_velocity.x();
  hw_state_imu_angular_velocity_[1] = angular_velocity.y();
  hw_state_imu_angular_velocity_[2] = angular_velocity.z();

  hw_state_imu_linear_acceleration_[0] = linear_acceleration.x();
  hw_state_imu_linear_acceleration_[1] = linear_acceleration.y();
  hw_state_imu_linear_acceleration_[2] = linear_acceleration.z();

  // Print the IMU
  // RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "IMU: %f, %f, %f, %f, %f, %f,
  // %f, %f, %f, %f",
  //     imu_output_.quat.x(), imu_output_.quat.y(), imu_output_.quat.z(), imu_output_.quat.w(),
  //     imu_output_.acc.x(), imu_output_.acc.y(), imu_output_.acc.z(),
  //     imu_output_.gyro.x(), imu_output_.gyro.y(), imu_output_.gyro.z());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!mujoco_interactive::is_robot_calibrated()) {
    RCLCPP_ERROR(rclcpp::get_logger("MujocoHardwareInterface"),
                 "Robot not calibrated. Skipping hw interface write");
    return hardware_interface::return_type::OK;
  }
  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"),
              "WRITE. com_pos: %f %f %f | %f %f %f | %f %f %f | %f %f %f",
              hw_command_positions_.at(0), hw_command_positions_.at(1), hw_command_positions_.at(2),
              hw_command_positions_.at(3), hw_command_positions_.at(4), hw_command_positions_.at(5),
              hw_command_positions_.at(6), hw_command_positions_.at(7), hw_command_positions_.at(8),
              hw_command_positions_.at(9), hw_command_positions_.at(10),
              hw_command_positions_.at(11));
  mujoco_interactive::ActuatorCommand command =
      mujoco_interactive::zero_command(info_.joints.size());
  for (int i = 0; i < info_.joints.size(); i++) {
    command.kp = hw_command_kps_;
    command.kd = hw_command_kds_;

    command.position_target = hw_command_positions_;
    command.velocity_target = hw_command_velocities_;
    command.feedforward_torque = hw_command_efforts_;
  }
  mujoco_interactive::set_actuator_command(command);
  return hardware_interface::return_type::OK;
}

}  // namespace mujoco_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_hardware_interface::MujocoHardwareInterface,
                       hardware_interface::SystemInterface)