/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "ros_gz_example_gazebo/FullSystem.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    ros_gz_example_gazebo::FullSystem,
    gz::sim::System,
    ros_gz_example_gazebo::FullSystem::ISystemConfigure,
    ros_gz_example_gazebo::FullSystem::ISystemPreUpdate,
    ros_gz_example_gazebo::FullSystem::ISystemUpdate,
    ros_gz_example_gazebo::FullSystem::ISystemPostUpdate,
    ros_gz_example_gazebo::FullSystem::ISystemReset
)

namespace ros_gz_example_gazebo 
{

void FullSystem::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  gzdbg << "ros_gz_example_gazebo::FullSystem::Configure on entity: " << _entity << std::endl;
}

void FullSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "ros_gz_example_gazebo::FullSystem::PreUpdate" << std::endl;
  }
}

void FullSystem::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "ros_gz_example_gazebo::FullSystem::Update" << std::endl;
  }
}

void FullSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "ros_gz_example_gazebo::FullSystem::PostUpdate" << std::endl;
  }
}

void FullSystem::Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm)
{
  gzdbg << "ros_gz_example_gazebo::FullSystem::Reset" << std::endl;
}
}  // namespace ros_gz_example_gazeba
