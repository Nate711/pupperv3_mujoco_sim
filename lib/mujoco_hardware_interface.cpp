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
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
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
  // TODO(nathan-kau) zero out positions etc
  // copies from weird spi struct to hw interface structs
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // TODO replace with calibrate motors blocking?
  do_homing();

  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Disable actuators
  // TODO disable

  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoHardwareInterface::read(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration &period) {
  // Write to and read from the actuators
  // TODO get act pos and vel from mujoco

  // Print joint position
  // RCLCPP_INFO(rclcpp::get_logger( MujocoHardwareInterface"), "Joint 0: %f",
  // spi_data_->q_abad[1]);

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
  copy_actuator_commands(true);
  // TODO write to mujoco pos commands
  return hardware_interface::return_type::OK;
}

void MujocoHardwareInterface::do_homing() {
  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Homing actuators...");
  // TODO calibrate motors here or just delete this func
  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Finished homing!");
}

void MujocoHardwareInterface::copy_actuator_commands(bool use_position_limits) {
  // copies from hw_command_blah to spi controller variables
  // TODO?
}

void MujocoHardwareInterface::copy_actuator_states() {
  // write to hw_state_positions_ and hw_state_velocities_
  // TODO?
}

}  // namespace mujoco_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_hardware_interface::MujocoHardwareInterface,
                       hardware_interface::SystemInterface)