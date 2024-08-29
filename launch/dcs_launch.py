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

import argparse
import os
import os.path
import sys

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

from webots_ros2_driver.webots_launcher import WebotsLauncher


def gen_exe_path(path) -> str:
    """Decorate executables with .exe suffix on windows platform."""
    if os.name == 'nt':
        path = path + '.exe'

    return path


def arg_to_bool(arg):
    """Support parsing of bool arguments."""
    if isinstance(arg, bool):
        return arg

    if arg.lower() in ('true', 'yes', '1', 'ok'):
        return True

    return False


def handle_arguments(args: list[str]) -> argparse.Namespace:
    """Process launcher specific arguments."""
    package_dir = get_package_share_directory('ros2_dcs_turtlesim')

    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument(
        '--world',
        action='store',
        type=str,
        required=False,
        default=os.path.join(package_dir, 'worlds',  'minimal', 'DcsMinimal.wbt')
    )

    arg_parser.add_argument(
        '--launch_webots',
        type=arg_to_bool,
        nargs='?',
        const=True,
        default=True,
        required=False,
        help='Enable/disble launch of Webots process.'
    )

    arg_parser.add_argument(
        '--launch_ru',
        type=arg_to_bool,
        nargs='?',
        const=True,
        default=True,
        required=False,
        help='Enable/disable launch of RadonUlzer process.'
    )

    arg_parser.add_argument(
        '--launch_dcs',
        type=arg_to_bool,
        nargs='?',
        const=True,
        default=True,
        required=False,
        help='Enable/disable launch of DroidControlShip process.'
    )

    arg_parser.add_argument(
        '--log_redirect',
        type=arg_to_bool,
        nargs='?',
        const=True,
        default=False,
        required=False,
    )

    python_args = [f"--{arg.replace(':=', '=')}" for arg in args]  # ros2 arg:=val to --arg=val
    return arg_parser.parse_args(python_args)


def generate_launch_description():
    """Launch all components necessary for this demo."""
    my_args = handle_arguments(sys.argv[4:])  # 4th+  are the launcher specifc.

    dcs_home = os.getenv('DCS_HOME')
    ru_home = os.getenv('RU_HOME')

    # sanity checks ...
    #
    if dcs_home is None:
        sys.exit('DCS_HOME environment variable not set.')
    if ru_home is None:
        sys.exit('RU_HOME environment variable not set.')

    wb_ctrl_path = gen_exe_path(
        os.path.join(
            get_package_share_directory('webots_ros2_driver'),
            'scripts',
            'webots-controller')
    )

    if not os.path.isfile(wb_ctrl_path):
        sys.exit(f'webots_controller program not found in {wb_ctrl_path}')

    dcs_path = gen_exe_path(
        os.path.join(dcs_home, '.pio', 'build', 'TurtleSim', 'program'))
    if not os.path.isfile(dcs_path):
        sys.exit(f'DroidControlShip controller not found in {dcs_path}')

    ru_path = gen_exe_path(
        os.path.join(ru_home, '.pio', 'build', 'RemoteControlSim', 'program'))
    if not os.path.isfile(ru_path):
        sys.exit(f'RadonUlzer controller not found in {ru_path}')

    webots = None
    if my_args.launch_webots:
        # webots with predefined world
        #
        webots = WebotsLauncher(world=my_args.world)

    wb_controller_cmd = [wb_ctrl_path]
    if my_args.log_redirect:
        wb_controller_cmd += ['--stdout-redirect', '--stderr-redirect']

    # DroidControlship Launcher
    #
    dcs_controller = ExecuteProcess(
        cmd=wb_controller_cmd + [
            '--robot-name=ZumoComSystem',
            dcs_path,
            '--cfgFilePath',
            os.path.join(dcs_home, 'data', 'config', 'config.json')
        ],
        name='DroidControlShip'
    )

    # RadonUlzer Launcher
    #
    ru_controller = ExecuteProcess(
        cmd=wb_controller_cmd + [
            '--robot-name=Zumo',
            ru_path,
        ],
        name='RadonUlzerRC'
    )

    # construct launch description
    #
    actions = []
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

    if my_args.launch_dcs:
        actions.append(dcs_controller)

    if my_args.launch_ru:
        actions.append(ru_controller)

    return LaunchDescription(actions)
