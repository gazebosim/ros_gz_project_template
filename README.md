# ros_gz_project_template
A template project integrating ROS and Gazebo simulator

## Included packages

* `ros_gz_example_description` - holds the sdf description of the simulated system and any other assets

* `ros_gz_example_gazebo` - holds gazebo specific code and configurations.  Namely this is where systems end up.

* `ros_gz_example_application` - holds ros2 specific code and configurations

* `ros_gz_example_bringup` - holds launch files and high level utilities


## Install
### Use as template
Directly `Use this template` and create your project repository on github.

### On Host System
##### Requirements
On Ubuntu Jammy

1. Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)

1. Install [Gazebo Garden](https://gazebosim.org/docs/garden)

1. Install necessary tools

    `sudo apt install python3-vcstool python3-colcon-common-extensions git wget`

##### Usage

1. Create a workspace, for example:

    ```
    mkdir -p ~/template_ws/src
    cd ~/template_ws/src
    ```

1. Clone the template:

    ```
    git clone 
    wget https://raw.githubusercontent.com/gazebosim/ros_gz_project_template/main/template_workspace.yaml
    vcs import < template_workspace.yaml
    cd ~/template_ws
    ```

1. Set the Gazebo version to Garden:

    ```
    export GZ_VERSION=garden
    ```

1. Install ROS dependencies

    ```
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y -i
    ```

1. Build and install

    ```
    cd ~/template_ws
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

##### Run

1. Source the workspace

    `. ~/template_ws/install/setup.sh`

1. Launch the simulation

    `ros2 launch ros_gz_example_bringup example.launch.py`
