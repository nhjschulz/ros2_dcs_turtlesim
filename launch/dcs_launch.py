"""DroidControlShip Launch Configuration for Webots."""
# MIT License
#
# Copyright (c) 2024 Norbert Schulz
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import os.path
import sys

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration

from webots_ros2_driver.webots_launcher import WebotsLauncher


def _gen_exe_path(path) -> str:
    """Decorate executables with .exe suffix on windows platform."""
    if os.name == 'nt':
        path = path + '.exe'

    return path


def _arg_to_bool(arg):
    """Support parsing of bool arguments."""
    if isinstance(arg, bool):
        return arg

    if arg.lower() in ('true', 'yes', '1', 'ok'):
        return True

    return False


def _define_launch_args() -> list[DeclareLaunchArgument]:
    """Get launcher arguments definitions."""
    return [
        DeclareLaunchArgument(
            name='world',
            default_value=os.path.join(
                get_package_share_directory('ros2_dcs_turtlesim'),
                'worlds',  'minimal', 'DcsMinimal.wbt'
            ),
            description='Path to Webots world file'
        ),
        DeclareLaunchArgument(
            name='launch_webots',
            default_value='true',
            description='Enable/disable launch of Webots process.'
        ),
        DeclareLaunchArgument(
            name='launch_dcs',
            default_value='true',
            description='Enable/disable launch of DroidControlShip process.'
        ),
        DeclareLaunchArgument(
            name='launch_ru',
            default_value='true',
            description='Enable/disable launch of RadonUlzer process.'
        ),
        DeclareLaunchArgument(
            name='launch_xrce',
            default_value='false',
            description='Enable/disable launch of the XRCE agent process.'
        ),
        DeclareLaunchArgument(
            name='xrce_agent',
            default_value='/usr/local/bin/MicroXRCEAgent udp4 -p 1883',
            description='Define command line for XRCE agent.'
        ),
        DeclareLaunchArgument(
            name='log_redirect',
            default_value='false',
            description='Enable/disable output redirection of Webots controllers.'
        )
    ]


def _launch_setup(context) -> list[LaunchDescriptionEntity]:
    """
    Generate launch item list forr launch context.

    See https://github.com/ros2/launch/blob/rolling/launch/launch/actions/opaque_function.py.
    """
    dcs_home = os.getenv('DCS_HOME')
    ru_home = os.getenv('RU_HOME')

    # sanity checks ...
    #
    if dcs_home is None:
        sys.exit('DCS_HOME environment variable not set.')
    if ru_home is None:
        sys.exit('RU_HOME environment variable not set.')

    wb_ctrl_path = _gen_exe_path(
        os.path.join(
            get_package_share_directory('webots_ros2_driver'),
            'scripts',
            'webots-controller')
    )

    if not os.path.isfile(wb_ctrl_path):
        sys.exit(f'webots_controller program not found in {wb_ctrl_path}')

    dcs_path = _gen_exe_path(
        os.path.join(dcs_home, '.pio', 'build', 'TurtleSim', 'program'))
    if not os.path.isfile(dcs_path):
        sys.exit(f'DroidControlShip controller not found in {dcs_path}')

    ru_path = _gen_exe_path(
        os.path.join(ru_home, '.pio', 'build', 'RemoteControlSim', 'program'))
    if not os.path.isfile(ru_path):
        sys.exit(f'RadonUlzer controller not found in {ru_path}')

    xrce_agent = ExecuteProcess(
            name='XRCE-Agent',
            cmd=LaunchConfiguration('xrce_agent').perform(context).split(),
            condition=IfCondition(LaunchConfiguration('launch_xrce'))
    )

    webots = None
    if _arg_to_bool(LaunchConfiguration('launch_webots').perform(context)):
        # webots with predefined world
        #
        webots = WebotsLauncher(world=LaunchConfiguration('world').perform(context))

    wb_controller_cmd = [wb_ctrl_path]
    if _arg_to_bool(LaunchConfiguration('log_redirect').perform(context)):
        wb_controller_cmd += ['--stdout-redirect', '--stderr-redirect']

    # DroidControlship Launcher
    #
    dcs_controller = ExecuteProcess(
        name='DroidControlShip',
        cmd=wb_controller_cmd + [
            '--robot-name=ZumoComSystem',
            dcs_path,
            '--cwd',
            dcs_home,
            '--cfgFilePath',
            os.path.join(dcs_home, 'data', 'config', 'config.json')
        ],
        condition=IfCondition(LaunchConfiguration('launch_dcs'))
    )

    # RadonUlzer Launcher
    #
    ru_controller = ExecuteProcess(
        name='RadonUlzerRC',
        cmd=wb_controller_cmd + [
            '--robot-name=Zumo',
            ru_path,
            '--cwd',
            ru_home,
            '-c',
            '-v'
        ],
        condition=IfCondition(LaunchConfiguration('launch_ru'))
    )

    # construct launch description
    #
    actions = [xrce_agent]

    if webots:
        actions.append(webots)
        actions.append(
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            )
        )

    actions.append(dcs_controller)
    actions.append(ru_controller)

    return actions


def generate_launch_description() -> LaunchDescription:
    """Create launch description list."""
    lds = LaunchDescription(_define_launch_args())
    lds.add_action(OpaqueFunction(function=_launch_setup))

    return lds
