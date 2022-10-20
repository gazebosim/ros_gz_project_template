// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef ROS_SYSTEM__ROSSYSTEM_HPP_
#define ROS_SYSTEM__ROSSYSTEM_HPP_

#include <memory>

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/System.hh>

#include <rclcpp/executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/static_single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros_gz_example_gazebo
{
// This is the main plugin's class. It must inherit from System and at least
// one other interface.
class RosSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  // Plugins inheriting ISystemConfigure must implement the Configure
  // callback. This is called when a system is initially loaded.
  // The _entity variable contains the entity that the system is attached to
  // The _element variable contains the sdf Element with custom configuration
  // The _ecm provides an interface to all entities and components
  // The _eventManager provides a mechanism for registering internal signals
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & element,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventManager) override;

  // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
  // callback. This is called at every simulation iteration after the physics
  // updates the world. The _info variable provides information such as time,
  // while the _ecm provides an interface to all entities and components in
  // simulation.
  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override;

  /// Callback when ROS message is received
  void OnStringMessage(const std_msgs::msg::String & msg);

private:
  /// Pointer to rclcpp Node
  rclcpp::Node::SharedPtr node_ {nullptr};

  /// Pointer to rclcpp Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr talker_pub_ {nullptr};

  /// Pointer to rclcpp Subscription
  rclcpp::SubscriptionBase::SharedPtr listener_sub_ {nullptr};
};
}  // namespace ros_gz_example_gazebo
#endif  // ROS_SYSTEM__ROSSYSTEM_HPP_
