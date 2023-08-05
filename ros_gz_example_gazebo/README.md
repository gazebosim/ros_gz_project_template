# ros_gz_example_gazebo

This subfolder holds example source files and a `CMakeLists.txt` file, as a starting point for Gazebo implementations outside of the official Gazebo repositories.

The provided `CMakeLists.txt` file contains the directives to compile two example Gazebo systems: `BasicSystem` and `FullSystem`.

For more information on Gazebo sim systems, see following [Gazebo Sim tutorials](https://gazebosim.org/api/sim/7/tutorials.html):

- [Create System Plugins](https://gazebosim.org/api/sim/7/createsystemplugins.html)
- [Migration from Gazebo Classic: Plugins](https://gazebosim.org/api/sim/7/migrationplugins.html)


## `BasicSystem` and `FullSystem`

`BasicSystem` is an example system that implements only the `ISystemPostUpdate` interface:

```c++
 class BasicSystem:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
```

`FullSystem` is an example system that implements all of the system interfaces:

```c++
class FullSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
```

See the comments in the sourece files for further documentation.

## `CMakeLists.txt`

See the comments in the `CMakeLists.txt` file for further documentation.