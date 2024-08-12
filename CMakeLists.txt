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

ament_package()


if (NOT DEFINED ENV{DCS_HOME})
  message(FATAL_ERROR "Environment variable DCS_HOME not set. Please define it with path to DCS location.")
else()
  message(STATUS "DCS_HOME is " $ENV(DCS_HOME))
endif()

# DCS build is based on PlatformIO
add_custom_target(DroidControlShip ALL
  COMMAND  $ENV{HOME}/.platformio/penv/bin/platformio run -e TurtleSim
  WORKING_DIRECTORY  $ENV{DCS_HOME}
)
