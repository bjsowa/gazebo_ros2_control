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

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "gazebo_ros2_control/gazebo_system.hpp"

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"
#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/SensorManager.hh"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace gazebo_ros2_control
{

bool GazeboSystem::initSim(
  rclcpp::Node::SharedPtr & ros_node,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr sdf)
{
  last_update_sim_time_ros_ = rclcpp::Time();

  ros_node_ = ros_node;
  parent_model_ = parent_model;

  gazebo::physics::PhysicsEnginePtr physics = parent_model_->GetWorld()->Physics();

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(ros_node->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  return true;
}

CallbackReturn GazeboSystem::on_init(const hardware_interface::HardwareInfo & system_info)
{
  info_ = system_info;

  registerJoints(info_);
  registerSensors(info_);

  if (n_dof_ == 0 && n_sensors_ == 0) {
    RCLCPP_WARN_STREAM(ros_node_->get_logger(), "There is no joint or sensor available");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void GazeboSystem::registerJoints(
  const hardware_interface::HardwareInfo & hardware_info)
{
  n_dof_ = hardware_info.joints.size();

  joint_names_.resize(n_dof_);
  joint_available_control_methods_.resize(n_dof_);
  joint_current_control_method_.resize(n_dof_, NONE);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_position_cmd_.resize(n_dof_);
  joint_velocity_cmd_.resize(n_dof_);
  joint_effort_cmd_.resize(n_dof_);

  for (unsigned int j = 0; j < n_dof_; j++) {
    std::string joint_name = joint_names_[j] = hardware_info.joints[j].name;

    gazebo::physics::JointPtr simjoint = parent_model_->GetJoint(joint_name);
    sim_joints_.push_back(simjoint);
    if (!simjoint) {
      RCLCPP_WARN_STREAM(
        ros_node_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Loading joint: " << joint_name);

    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < hardware_info.joints[j].command_interfaces.size(); i++) {
      if (hardware_info.joints[j].command_interfaces[i].name ==
        hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t\t position");
        joint_available_control_methods_[j].push_back(POSITION);
        command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &joint_position_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name ==
        hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t\t velocity");
        joint_available_control_methods_[j].push_back(VELOCITY);
        command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &joint_velocity_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name ==
        hardware_interface::HW_IF_EFFORT)
      {
        joint_available_control_methods_[j].push_back(EFFORT);
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t\t effort");
        command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &joint_effort_cmd_[j]);
      }
    }

    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\tState:");

    // register the state handles
    for (unsigned int i = 0; i < hardware_info.joints[j].state_interfaces.size(); i++) {
      if (hardware_info.joints[j].state_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t\t position");
        state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &joint_position_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t\t velocity");
        state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &joint_velocity_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == hardware_interface::HW_IF_EFFORT) {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t\t effort");
        state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &joint_effort_[j]);
      }
    }
  }
}

void GazeboSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // Collect gazebo sensor handles
  size_t n_sensors = hardware_info.sensors.size();
  std::vector<hardware_interface::ComponentInfo> imu_components_;
  std::vector<hardware_interface::ComponentInfo> ft_sensor_components_;

  // This is split in two steps: Count the number and type of sensor and associate the interfaces
  // So we have resize only once the structures where the data will be stored, and we can safely
  // use pointers to the structures
  for (unsigned int j = 0; j < n_sensors; j++) {
    hardware_interface::ComponentInfo component = hardware_info.sensors[j];
    std::string sensor_name = component.name;

    // This can't be used, because it only looks for sensor in links, but force_torque_sensor
    // must be in a joint, as in not found by SensorScopedName
    // std::vector<std::string> gz_sensor_names = parent_model_->SensorScopedName(sensor_name);

    // Workaround to find sensors whose parent is a link or joint of parent_model
    std::vector<std::string> gz_sensor_names;
    for (const auto & s : gazebo::sensors::SensorManager::Instance()->GetSensors()) {
      const std::string sensor_parent = s->ParentName();
      if (s->Name() != sensor_name) {
        continue;
      }
      if ((parent_model_->GetJoint(sensor_parent) != nullptr) ||
        parent_model_->GetLink(sensor_parent) != nullptr)
      {
        gz_sensor_names.push_back(s->ScopedName());
      }
    }
    if (gz_sensor_names.empty()) {
      RCLCPP_WARN_STREAM(
        ros_node_->get_logger(), "Skipping sensor in the URDF named '" << sensor_name <<
          "' which is not in the gazebo model.");
      continue;
    }
    if (gz_sensor_names.size() > 1) {
      RCLCPP_WARN_STREAM(
        ros_node_->get_logger(), "Sensor in the URDF named '" << sensor_name <<
          "' has more than one gazebo sensor with the " <<
          "same name, only using the first. It has " << gz_sensor_names.size() << " sensors");
    }

    gazebo::sensors::SensorPtr simsensor = gazebo::sensors::SensorManager::Instance()->GetSensor(
      gz_sensor_names[0]);
    if (!simsensor) {
      RCLCPP_ERROR_STREAM(
        ros_node_->get_logger(),
        "Error retrieving sensor '" << sensor_name << " from the sensor manager");
      continue;
    }
    if (simsensor->Type() == "imu") {
      gazebo::sensors::ImuSensorPtr imu_sensor =
        std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(simsensor);
      if (!imu_sensor) {
        RCLCPP_ERROR_STREAM(
          ros_node_->get_logger(),
          "Error retrieving casting sensor '" << sensor_name << " to ImuSensor");
        continue;
      }
      imu_components_.push_back(component);
      sim_imu_sensors_.push_back(imu_sensor);
    } else if (simsensor->Type() == "force_torque") {
      gazebo::sensors::ForceTorqueSensorPtr ft_sensor =
        std::dynamic_pointer_cast<gazebo::sensors::ForceTorqueSensor>(simsensor);
      if (!ft_sensor) {
        RCLCPP_ERROR_STREAM(
          ros_node_->get_logger(),
          "Error retrieving casting sensor '" << sensor_name << " to ForceTorqueSensor");
        continue;
      }
      ft_sensor_components_.push_back(component);
      sim_ft_sensors_.push_back(ft_sensor);
    }
  }

  imu_sensor_data_.resize(sim_imu_sensors_.size());
  ft_sensor_data_.resize(sim_ft_sensors_.size());
  n_sensors_ = sim_imu_sensors_.size() +
    sim_ft_sensors_.size();

  for (unsigned int i = 0; i < imu_components_.size(); i++) {
    const std::string & sensor_name = imu_components_[i].name;
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Loading sensor: " << sensor_name);
    RCLCPP_INFO_STREAM(
      ros_node_->get_logger(), "\tState:");
    for (const auto & state_interface : imu_components_[i].state_interfaces) {
      static const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0},
        {"orientation.y", 1},
        {"orientation.z", 2},
        {"orientation.w", 3},
        {"angular_velocity.x", 4},
        {"angular_velocity.y", 5},
        {"angular_velocity.z", 6},
        {"linear_acceleration.x", 7},
        {"linear_acceleration.y", 8},
        {"linear_acceleration.z", 9},
      };
      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t\t " << state_interface.name);

      size_t data_index = interface_name_map.at(state_interface.name);
      state_interfaces_.emplace_back(
        sensor_name,
        state_interface.name,
        &imu_sensor_data_[i][data_index]);
    }
  }
  for (unsigned int i = 0; i < ft_sensor_components_.size(); i++) {
    const std::string & sensor_name = ft_sensor_components_[i].name;
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Loading sensor: " << sensor_name);
    RCLCPP_INFO_STREAM(
      ros_node_->get_logger(), "\tState:");
    for (const auto & state_interface : ft_sensor_components_[i].state_interfaces) {
      static const std::map<std::string, size_t> interface_name_map = {
        {"force.x", 0},
        {"force.y", 1},
        {"force.z", 2},
        {"torque.x", 3},
        {"torque.y", 4},
        {"torque.z", 5}
      };
      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t\t " << state_interface.name);

      size_t data_index = interface_name_map.at(state_interface.name);
      state_interfaces_.emplace_back(
        sensor_name,
        state_interface.name,
        &ft_sensor_data_[i][data_index]);
    }
  }
}

std::vector<hardware_interface::StateInterface>
GazeboSystem::export_state_interfaces()
{
  return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
GazeboSystem::export_command_interfaces()
{
  return std::move(command_interfaces_);
}

CallbackReturn GazeboSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GazeboSystem::read()
{
  for (unsigned int j = 0; j < joint_names_.size(); j++) {
    if (sim_joints_[j]) {
      joint_position_[j] = sim_joints_[j]->Position(0);
      joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      joint_effort_[j] = sim_joints_[j]->GetForce(0u);
    }
  }

  for (unsigned int j = 0; j < sim_imu_sensors_.size(); j++) {
    auto sim_imu = sim_imu_sensors_[j];
    imu_sensor_data_[j][0] = sim_imu->Orientation().X();
    imu_sensor_data_[j][1] = sim_imu->Orientation().Y();
    imu_sensor_data_[j][2] = sim_imu->Orientation().Z();
    imu_sensor_data_[j][3] = sim_imu->Orientation().W();

    imu_sensor_data_[j][4] = sim_imu->AngularVelocity().X();
    imu_sensor_data_[j][5] = sim_imu->AngularVelocity().Y();
    imu_sensor_data_[j][6] = sim_imu->AngularVelocity().Z();

    imu_sensor_data_[j][7] = sim_imu->LinearAcceleration().X();
    imu_sensor_data_[j][8] = sim_imu->LinearAcceleration().Y();
    imu_sensor_data_[j][9] = sim_imu->LinearAcceleration().Z();
  }

  for (unsigned int j = 0; j < sim_ft_sensors_.size(); j++) {
    auto sim_ft_sensor = sim_ft_sensors_[j];
    imu_sensor_data_[j][0] = sim_ft_sensor->Force().X();
    imu_sensor_data_[j][1] = sim_ft_sensor->Force().Y();
    imu_sensor_data_[j][2] = sim_ft_sensor->Force().Z();

    imu_sensor_data_[j][3] = sim_ft_sensor->Torque().X();
    imu_sensor_data_[j][4] = sim_ft_sensor->Torque().Y();
    imu_sensor_data_[j][5] = sim_ft_sensor->Torque().Z();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::write()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->SimTime();
  rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  for (unsigned int j = 0; j < joint_names_.size(); j++) {
    if (sim_joints_[j]) {
      if (joint_current_control_method_[j] == POSITION) {
        sim_joints_[j]->SetPosition(0, joint_position_cmd_[j], true);
      } else if (joint_current_control_method_[j] == VELOCITY) {
        sim_joints_[j]->SetVelocity(0, joint_velocity_cmd_[j]);
      } else if (joint_current_control_method_[j] == EFFORT) {
        sim_joints_[j]->SetForce(0, joint_effort_cmd_[j]);
      }
    }
  }

  last_update_sim_time_ros_ = sim_time_ros;

  return hardware_interface::return_type::OK;
}

}  // namespace gazebo_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gazebo_ros2_control::GazeboSystem, gazebo_ros2_control::GazeboSystemInterface)
