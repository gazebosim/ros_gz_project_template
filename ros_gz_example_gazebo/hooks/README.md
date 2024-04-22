# Hooks

The `ament_environment_hooks` is a feature of `ament` that allows users to set arbitrary environment variables as part of sourcing the environment. It is briefly documented [here](https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html#adding-to-the-ament-index).

In practice, this means that variables that are set in that folder are set when you call `setup.sh` or are set when dependent packages are being built.

These are populated and installed as part of these function calls:

```txt
 # The following hooks are used to ensure that the correct environment variables 
 # will be set by executing 'source install/setup.bash' after compilation. 
 # When using this template for your project, change the filenames of the 
 # files in the 'hooks' folder, to correspond to your project name. 
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")
```

where the `.in` suffix causes them to be pre-processed by CMake's [configure_file](https://cmake.org/cmake/help/latest/command/configure_file.html) call.

The actual implementation of `ament_environment_hook` is [here](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/cmake/environment_hooks/ament_environment_hooks.cmake) if that is also helpful in understanding.

Specifically for this project, we set the environment variables:

- `GZ_SIM_RESOURCE_PATH` - this appends the paths to world and model files such Gazebo can find them at runtime
- `GZ_SIM_SYSTEM_PLUGIN_PATH` - this appends the paths to system plugin shared libraries such that Gazebo can find them at runtime.

The two template files are:

- `.sh` - This will be evaluated by Linux/macOS shells (sh, bash, zsh, etc).
- `.dsv` - This is a machine-readable format of the expected environment changes that ament will take advantage of for performance (faster than sourcing shell scripts, in general). You can read more about this here: [generate .dsv files beside known environment hooks ament/ament_cmake#187](https://github.com/ament/ament_cmake/pull/187) and here: [improve performance when setting up environment ros2/ros2#764](https://github.com/ros2/ros2/issues/764)