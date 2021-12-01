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


#ifndef GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_INTERFACE_HPP_
#define GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_INTERFACE_HPP_

#include "gazebo/physics/PhysicsTypes.hh"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/node.hpp"

#include "sdf/Element.hh"

namespace gazebo_ros2_control
{

// SystemInterface provides API-level access to read and command joint properties.
class GazeboSystemInterface
  : public hardware_interface::SystemInterface
{
public:
  /// \brief Initilize the system interface
  /// param[in] ros_node pointer to the ros2 node
  /// param[in] parent_model pointer to the model
  /// param[in] control_hardware vector filled with information about robot's control resources
  /// param[in] sdf pointer to the SDF
  virtual bool initSim(
    rclcpp::Node::SharedPtr & ros_node,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) = 0;
};

}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_INTERFACE_HPP_
