cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(Python_FIND_VIRTUALENV "ONLY")
set(Python_ROOT_DIR "/home/grant/miniforge3/envs/penguinpi_env/bin/python3.10")
# Also tried Python_FIND_ABI, Python_LIBRARY, Python_INCLUDE_DIR

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# need custom find_package for python3
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter Development)
# Python_FOUND, Python_Interpreter_FOUND, Python_EXECUTABLE, Python_STDLIB, Python_STDARCH, Python_SITELIB, Python_SITEARCH, Python_SOABI, Python_Development_FOUND, Python_INCLUDE_DIRS
# message(STATUS "Python_FIND_ABI is set to: ${Python_FIND_ABI}")
# message(STATUS "Python_FOUND is set to: ${Python_FOUND}")
# message(STATUS "Python_Interpreter_FOUND is set to: ${Python_Interpreter_FOUND}")
# message(STATUS "Python_EXECUTABLE is set to: ${Python_EXECUTABLE}")
# message(STATUS "Python_STDLIB is set to: ${Python_STDLIB}")
# message(STATUS "Python_STDARCH is set to: ${Python_STDARCH}")
# message(STATUS "Python_SITELIB is set to: ${Python_SITELIB}")
# message(STATUS "Python_SITEARCH is set to: ${Python_SITEARCH}")
# message(STATUS "Python_SOABI is set to: ${Python_SOABI}")
# message(STATUS "Python_Development_FOUND is set to: ${Python_Development_FOUND}")
# message(STATUS "Python_INCLUDE_DIRS is set to: ${Python_INCLUDE_DIRS}")

## COMPILE
add_library(
  ros2_control_demo_example_2
  SHARED
  hardware/penguinpi_comms.cpp
  hardware/diffbot_system.cpp
)
target_compile_features(ros2_control_demo_example_2 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_2 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_2>
)
ament_target_dependencies(
  ros2_control_demo_example_2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Python3
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_2.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_2
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_2
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_2
)
install(TARGETS ros2_control_demo_example_2
  EXPORT export_ros2_control_demo_example_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
