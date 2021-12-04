// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_
#define GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_

#include "gazebo_ros2_control/gazebo_system_interface.hpp"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"

namespace gazebo_ros2_control
{

// These class must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

class GazeboSystem : public GazeboSystemInterface
{
public:
  // Documentation Inherited
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type read() override;

  // Documentation Inherited
  hardware_interface::return_type write() override;

  // Documentation Inherited
  bool initSim(
    rclcpp::Node::SharedPtr & ros_node,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

private:
  // Methods used to control a joint.
  enum ControlMethod
  {
    NONE,
    POSITION,
    VELOCITY,
    EFFORT,
  };

  void registerJoints(
    const hardware_interface::HardwareInfo & hardware_info);

  void registerSensors(
    const hardware_interface::HardwareInfo & hardware_info);

  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief Number of sensors.
  size_t n_sensors_;

  // Pointer to ROS node
  rclcpp::Node::SharedPtr ros_node_;

  /// \brief Gazebo Model Ptr.
  gazebo::physics::ModelPtr parent_model_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<std::string> joint_names_;

  /// \brief vector with the control methods defined in the URDF for each joint.
  std::vector<std::vector<ControlMethod>> joint_available_control_methods_;

  /// \brief vector with current control method for each joint.
  std::vector<ControlMethod> joint_current_control_method_;

  /// \brief handles to the joints from within Gazebo
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  /// \brief vector with the current joint position
  std::vector<double> joint_position_;

  /// \brief vector with the current joint velocity
  std::vector<double> joint_velocity_;

  /// \brief vector with the current joint effort
  std::vector<double> joint_effort_;

  /// \brief vector with the current cmd joint position
  std::vector<double> joint_position_cmd_;

  /// \brief vector with the current cmd joint velocity
  std::vector<double> joint_velocity_cmd_;

  /// \brief vector with the current cmd joint effort
  std::vector<double> joint_effort_cmd_;

  /// \brief handles to the imus from within Gazebo
  std::vector<gazebo::sensors::ImuSensorPtr> sim_imu_sensors_;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::vector<std::array<double, 10>> imu_sensor_data_;

  /// \brief handles to the FT sensors from within Gazebo
  std::vector<gazebo::sensors::ForceTorqueSensorPtr> sim_ft_sensors_;

  /// \brief An array per FT sensor for 3D force and torquee
  std::vector<std::array<double, 6>> ft_sensor_data_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
};

}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_
