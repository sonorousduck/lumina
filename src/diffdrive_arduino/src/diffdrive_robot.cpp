#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <controller_manager/controller_manager.h>
#include "diffdrive_arduino/diffdrive_arduino.h"
#include <iostream>

int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create an executor and a node
  auto node = std::make_shared<rclcpp::Node>("diffdrive_robot");
  RCLCPP_INFO(node->get_logger(), "Starting diffdrive_robot node...");

  // Create a publisher
  auto voltage_publisher = node->create_publisher<std_msgs::msg::Float32>("battery/voltage", 10);

  // DiffDriveArduino configuration
  DiffDriveArduino::Config robot_cfg;

  // Attempt to retrieve parameters. If they don't exist, the default values from the struct will be used
  node->declare_parameter("left_wheel_name", robot_cfg.left_wheel_name);
  node->declare_parameter("right_wheel_name", robot_cfg.right_wheel_name);
  node->declare_parameter("baud_rate", robot_cfg.baud_rate);
  node->declare_parameter("device", robot_cfg.device);
  node->declare_parameter("enc_counts_per_rev", robot_cfg.enc_counts_per_rev);
  node->declare_parameter("robot_loop_rate", robot_cfg.loop_rate);

  node->get_parameter("left_wheel_name", robot_cfg.left_wheel_name);
  node->get_parameter("right_wheel_name", robot_cfg.right_wheel_name);
  node->get_parameter("baud_rate", robot_cfg.baud_rate);
  node->get_parameter("device", robot_cfg.device);
  node->get_parameter("enc_counts_per_rev", robot_cfg.enc_counts_per_rev);
  node->get_parameter("robot_loop_rate", robot_cfg.loop_rate);

  DiffDriveArduino robot(robot_cfg);
  controller_manager::ControllerManager cm(&robot);

  rclcpp::Rate loop_rate(robot_cfg.loop_rate);

  // Main loop
  while (rclcpp::ok())
  {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());
    robot.write();

    std_msgs::msg::Float32 msg;
    msg.data = robot.read_voltage();
    voltage_publisher->publish(msg);
    std::cout << robot.read_voltage() << std::endl;

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
