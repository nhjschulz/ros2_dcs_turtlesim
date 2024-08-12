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

cmake_minimum_required(VERSION 3.8)
project(ros2_dcs_turtlesim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if (NOT DEFINED ENV{DCS_HOME})
  message(FATAL_ERROR "Environment variable DCS_HOME not set. Please define it with path to DroidControlShip location.")
else()
  message(STATUS "DCS_HOME is " $ENV(DCS_HOME))
endif()

if (NOT DEFINED ENV{RU_HOME})
  message(FATAL_ERROR "Environment variable RU_HOME not set. Please define it with path to RadonUlzer location.")
else()
  message(STATUS "RU_HOME is " $ENV(DCS_HOME))
endif()

# DCS build is based on PlatformIO
add_custom_target(DroidControlShip ALL
  COMMAND  $ENV{HOME}/.platformio/penv/bin/platformio run -e TurtleSim
  WORKING_DIRECTORY  $ENV{DCS_HOME}
)

# RU build is based on PlatformIO
add_custom_target(RadonUlzer ALL
  COMMAND  $ENV{HOME}/.platformio/penv/bin/platformio run -e RemoteControlSim
  WORKING_DIRECTORY  $ENV{RU_HOME}
)

# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
