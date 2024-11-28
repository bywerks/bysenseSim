#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // Include JointState message
#include <iostream> // Include this header for std::cout
#include <algorithm> 
#include <sensor_msgs/msg/imu.hpp> 

#define RATE 1000
//ID order 

// sitting position when starting robot                    
// Axis ID                         2,    3,   4,   1,   8,    9,    6,   7,  10,   5,  11,   12
std::vector<double> setpoints = {1.5, -2.9, 0.0, 0.0, 1.5, -2.9, -2.9, 0.0, 0.0, 1.5, 1.5, -2.9};

// Global variable to store the last joint states
std::vector<double> last_joint_positions;
std::vector<double> last_joint_velocity;
std::shared_ptr<sensor_msgs::msg::JointState> joint_state_msg;

void setpoint_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  // Update the setpoints from the high-level controller (e.g., Python node)
  if (msg->data.size() == setpoints.size()) {
    for (size_t i = 0; i < setpoints.size(); ++i) {
      setpoints[i] = msg->data[i];
    }
  }
}


double calculate_torque(double current_position, double target_position, double velocity)
{
  double kp = 200.0;
  double kd = 1.5;
  double Torque_Limit = 80.0;

  double delta_position = target_position - current_position;
  double compensated_torque = kp * delta_position - kd * velocity;
  double clippedTorque= std::clamp(compensated_torque, -Torque_Limit, Torque_Limit);
  double fwd_torque = clippedTorque;
  return fwd_torque;
}


// Function to handle joint state callback
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Store the joint positions from the last received joint states
    last_joint_positions = msg->position;
    last_joint_velocity = msg->velocity;

    // Update the shared pointer to the latest message
    joint_state_msg = msg; // This is the change
}


int main(int argc, char * argv[])
{
  // ROS node initialization
  rclcpp::init(argc, argv);

  auto robot_controller_node = std::make_shared<rclcpp::Node>("robot_controller_node");

  // Subscribe to setpoints from the high-level controller (e.g., Python node)
  auto setpoint_sub = robot_controller_node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "high_level/setpoints", 10, setpoint_callback);

  // Subscribe to joint states
  auto joint_state_sub = robot_controller_node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, joint_state_callback);

  // Gazebo command publisher
  auto command_pub = robot_controller_node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "effort_controller/commands", rclcpp::SensorDataQoS());

  // Main loop rate (1000 Hz)
  rclcpp::Rate loop_rate(RATE);

  while (rclcpp::ok()) {

    // Prepare command message
    std_msgs::msg::Float64MultiArray command_message;
    command_message.layout.data_offset = 1;

    if (joint_state_msg != nullptr) {
      std::vector<double> positions = joint_state_msg->position;
      std::vector<double> velocities = joint_state_msg->velocity;
      std::vector<std::string> joint_names = joint_state_msg->name;

      for (size_t i = 0; i < setpoints.size(); ++i) {
        if (!std::isnan(setpoints[i])) {

          double current_position = positions[i];
          double velocity = velocities[i];
          double torque = calculate_torque(current_position, setpoints[i], velocity);
          command_message.data.push_back(torque);
        }
      }

    }

    else { }
    // Publish joint angle commands to Gazebo
    command_pub->publish(command_message);

    // Spin to process callbacks (IMU and setpoints)
    rclcpp::spin_some(robot_controller_node);

    // Sleep to maintain the loop rate (1000 Hz)
    loop_rate.sleep();
    
  }
  

  return 0;
}
