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

#ifndef ROS_GZ_EXAMPLE_GAZEBO__FULL_SYSTEM_HH_
#define ROS_GZ_EXAMPLE_GAZEBO__FULL_SYSTEM_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

namespace ros_gz_example_gazebo
{
  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class FullSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
  {
    // Plugins inheriting ISystemConfigure must implement the Configure 
    // callback. This is called when a system is initially loaded. 
    // The _entity variable contains the entity that the system is attached to
    // The _element variable contains the sdf Element with custom configuration
    // The _ecm provides an interface to all entities and components
    // The _eventManager provides a mechanism for registering internal signals
    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager) override;

    // Plugins inheriting ISystemPreUpdate must implement the PreUpdate
    // callback. This is called at every simulation iteration before the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemUpdate must implement the Update
    // callback. This is called at every simulation iteration before the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemReset must implement the Reset callback. 
    // This is called when simulation is reset/rewound to initial conditions.
    public: void Reset(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
  };
}
#endif
