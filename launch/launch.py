# MIT License
# 
# Copyright (c) 2024 Norbert Schulz
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import os.path
import sys
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def gen_exe_path(path) -> str:
    if os.name == 'nt':
       return path + ".exe"
    
    return path
    

def generate_launch_description():
    package_dir = get_package_share_directory('ros2_dcs_turtlesim')
    wb_ros_controller_dir = get_package_share_directory('webots_ros2_driver')


    dcs_home = os.getenv("DCS_HOME")
    ru_home = os.getenv("RU_HOME")

    # sanity checks ...
    #
    if dcs_home is None:
        sys.exit("DCS_HOME environment variable not set.")
    if ru_home is None:
        sys.exit("RU_HOME environment variable not set.")

    wb_controller_path = gen_exe_path(os.path.join(wb_ros_controller_dir, "scripts", "webots-controller"))
    if not os.path.isfile(wb_controller_path):
        sys.exit(f"webots_controller program not found in {wb_controller_path}")
    
    dcs_path = os.path.join(dcs_home, ".pio", "build", "TurtleSim", "program")
    if not os.path.isfile(dcs_path):
        sys.exit(f"DroidControlShip controller not found in {dcs_path}")

    ru_path = os.path.join(ru_home, ".pio", "build", "RemoteControlSim", "program")
    if not os.path.isfile(ru_path):
        sys.exit(f"RadonUlzer controller not found in {ru_path}")

    # webots with predefined world
    #
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'ZumoComSystemOnly.wbt')
    )

    # DroidControlship Launcher
    #
    dcs_controller = ExecuteProcess(
        cmd = 
        [
            wb_controller_path,
            "--robot-name=ZumoComSystem",
            dcs_path,
             "--cfgFilePath",
            os.path.join(dcs_home, "data", "config", "config.json")
        ], 
        name="DroidControlShip"
    )

    # RadonUlzer Launcher
    #
    ru_controller = ExecuteProcess(
        cmd = 
        [
            wb_controller_path,
            "--robot-name=Zumo",
            ru_path,
        ],
        name="RadonUlzer RC"
    )

    # Run Forest, run ....
    #
    return LaunchDescription([
        webots,
        dcs_controller,
        ru_controller,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
