#ifndef H_MUJOCO_NODE
#define H_MUJOCO_NODE

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include "actuator_model.hpp"
#include "mujoco_core_interactive.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class MujocoInteractiveNode : public rclcpp::Node {
 public:
  MujocoInteractiveNode(std::vector<std::string> joint_names,
                        std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models);
  ~MujocoInteractiveNode();
  void start_simulation();
  void calibrate_motors_detached();
  void run_gui_blocking();
  void run_gui_detached();

 private:
  void make_pub_subs(const float publish_rate);
  void allocate_messages(const std::vector<std::string> &joint_names,
                         const unsigned int &n_actuators);
  void joint_state_publish_callback();
  void joint_command_callback(const pupper_interfaces::msg::JointCommand &msg);
  static bool check_joint_command_is_valid(const pupper_interfaces::msg::JointCommand &msg,
                                           int n_actuators);

  // Mujoco physics calculation thread
  std::unique_ptr<std::thread> simulate_thread_;

  // Actuator models
  std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models_;
  std::vector<double> actuator_torques_;

  int n_actuators_;
  mujoco_interactive::ActuatorCommand command;

  // Joint State Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;
  sensor_msgs::msg::JointState joint_state_message_;
  std::vector<std::string> joint_names_;

  // Body state publisher
  std::unique_ptr<tf2_ros::TransformBroadcaster> body_tf_broadcaster_;
  geometry_msgs::msg::TransformStamped body_tf_;

  // Clock publisher
  // Publishes every physics step
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

  // Subscriber
  rclcpp::Subscription<pupper_interfaces::msg::JointCommand>::SharedPtr subscription_;
  pupper_interfaces::msg::JointCommand latest_msg_;
};

#endif