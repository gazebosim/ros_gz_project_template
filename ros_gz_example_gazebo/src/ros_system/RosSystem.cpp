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

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Include the plugin's header.
#include "RosSystem.hpp"

#include <std_msgs/msg/string.hpp>

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
  ros_gz_example_gazebo::RosSystem,
  gz::sim::System,
  ros_gz_example_gazebo::RosSystem::ISystemConfigure,
  ros_gz_example_gazebo::RosSystem::ISystemPostUpdate)

namespace ros_gz_example_gazebo
{

void RosSystem::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & element,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & eventManager)
{
  // Ensure that ROS is setup
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Read configuration from SDF file
  auto node_name = element->Get<std::string>("node_name", "RosSystem").first;
  auto talker_topic = element->Get<std::string>("talker_topic", "talker").first;
  auto listener_topic = element->Get<std::string>("listener_topic", "listener").first;

  node_ = rclcpp::Node::make_shared(node_name);
  listener_sub_ = node_->create_subscription<std_msgs::msg::String>(
    listener_topic,
    1, std::bind(&RosSystem::OnStringMessage, this, std::placeholders::_1));
  talker_pub_ = node_->create_publisher<std_msgs::msg::String>(talker_topic, 1);
}

void RosSystem::OnStringMessage(const std_msgs::msg::String & msg)
{
  gzmsg << "I heard: " << msg.data << std::endl;
}

void RosSystem::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  rclcpp::spin_some(node_);
  if (!_info.paused && _info.iterations % 1000 == 0) {
    // Gazebo debug output
    gzdbg << "ros_gz_example_gazebo::BasicSystem::PostUpdate" << std::endl;

    // Publish a message
    std_msgs::msg::String msg;
    msg.data = "ros_gz_example_gazebo::RosSystem::PostUpdate";
    talker_pub_->publish(msg);
  }
}

}  // namespace ros_gz_example_gazebo
