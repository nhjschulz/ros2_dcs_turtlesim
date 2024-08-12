import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('ros2_dcs_turtlesim')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'ZumoComSystemOnly.wbt')
    )

    dcs_controller = ExecuteProcess( 
        cmd = [os.path.join(os.getenv("DCS_HOME"), ".pio", "build", "TurtleSim", "program")],
        cwd = os.path.join(os.getenv("DCS_HOME"))
    )

    return LaunchDescription([
        webots,
        dcs_controller,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
