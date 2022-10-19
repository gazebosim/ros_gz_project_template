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

#ifndef BASIC_SYSTEM__BASICSYSTEM_HPP_
#define BASIC_SYSTEM__BASICSYSTEM_HPP_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/System.hh>

namespace ros_gz_example_gazebo
{
// This is the main plugin's class. It must inherit from System and at least
// one other interface.
// Here we use `ISystemPostUpdate`, which is used to get results after
// physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
// plugins that want to send commands.
class BasicSystem
  : public gz::sim::System,
  public gz::sim::ISystemPostUpdate
{
public:
  // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
  // callback. This is called at every simulation iteration after the physics
  // updates the world. The _info variable provides information such as time,
  // while the _ecm provides an interface to all entities and components in
  // simulation.
  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;
};
}  // namespace ros_gz_example_gazebo
#endif  // BASIC_SYSTEM__BASICSYSTEM_HPP_
