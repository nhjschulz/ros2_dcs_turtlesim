# ros2_dcs_turtlesim
This project is a ROS2 Package wrapper for the TurtleSim demo with
DroidControlShip (DCS) and RadonUlzer (RU) robots under Webots.

![ZumoTurtle](ZumoTurtle.jpg)

These robot projects are build using PlatformIO. The ros2_dcs_turtlesim
package creates a ROS2 cmake wrapper around them and adds a launch
configuration for the turtle demo.

## Workspace Setup

### Checkout `ros2_dcs_turtlesim` into your ROS2 workspace

Go into the src folder of your ros2 workspace and checkout this
repository there:

        cd ros2_webots_ws/src
        git clone https://github.com/nhjschulz/ros2_dcs_turtlesim

### Checkout the dependend PlatformIO projects

Checkout the following PlatformIO projects to a fitting location.
This should not be inside your ROS workspace, as these are not ROS2 
packages:

* RadonUlzer: https://github.com/BlueAndi/RadonUlzer
* DroidControlShip: https://github.com/BlueAndi/DroidControlShip (Branch feature/ROS2)

    
        git clone https://github.com/BlueAndi/RadonUlzer
        git clone https://github.com/BlueAndi/DroidControlShip
        cd DroidControlShip
        git checkout feature/ROS2

* Link the micro-ROS library to the lib folder of DCS

        cd DroidControlShip/lib
        ln -s ~/microros_ws/firmware/build libmicroros

Add the following environment variables which tell `ros2_dcs_turtlesim`
where the PlatformIO based projects are:

        export DCS_HOME=<checkout location of DroidControlShip>
        export RU_HOME=<checkout location of RadonUlzer>

## Building 

Go into your ROS2 workspace and use the normal ROS2 commands for building

        colcon build
        source install/local_setup.bash

The first build must be the complete workspace. Later it is possible to
build only this package (and in verbose mode) use:

        colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON  --packages-select ros2_dcs_turtlesim
        source install/local_setup.bash

> [!NOTE]
> If you get error ```fatal error: rcl/rcl.h: No such file or directory```, you forgot to link the micro-ROS library into
> the DCS workspace.

## Launching

The `ros2_dcs_turtlesim` includes a launch configuration. Run it using

        ros2 launch ros2_dcs_turtlesim dcs_launch.py [args]

> [!NOTE]
> The argument ```launch_xrce:=true``` is required unless the xrce agent is already running as a demon or in another terminal.

Supported launcher arguments:

| Argument      | Default              | Description          |
|---------------|----------------------|----------------------|
|world|world:=worlds/minimal/DcsMinimal.wbt| Launch Webots with given world file.|
|launch_webots|launch_webots:=true| Enable/Disable launch of Webots.|
|launch_ru|launch_ru:=true| Enable/Disable launch of RadonUlzer controller.|
|launch_dcs|launch_dcs:=true| Enable/Disable launch of DroidControlShip controller.|
|launch_xrce|launch_xrce:=false| Enable/disable launch of the XRCE agent process.|
|xrce_agent|xrce_agent:='/usr/local/bin/MicroXRCEAgent tcp4 -p 1883'|Define command line for XRCE agent.|
|log_redirect|log_redirect:=false| Enable/Disable output redirection of Webots controllers to Webots console.|

## Controlling the Robot

The robot listens to Twist messages similar to the ROS2 TurtleSim demo. The ```turtle_teleop_key``` tool can be used
to control it using the keyboard. Use the following command from a second terminal window for controlling the robot:

        ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=/cmd_vel
