cmake_minimum_required(VERSION 3.8)
project(exoskeleton_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 1. Tell CMake where to find the Beckhoff header files
include_directories(
  include
  AdsLib
)

# 2. Gather all the Beckhoff C++ source files
file(GLOB ADSLIB_SOURCES "AdsLib/*.cpp")

# 3. Compile your hardware interface AND the Beckhoff files into one shared library
add_library(TwinCatHardware SHARED
  src/twincat_system.cpp
  ${ADSLIB_SOURCES}
)

target_compile_definitions(TwinCatHardware PRIVATE CONFIG_DEFAULT_LOGLEVEL=3)

# 4. Link the ROS 2 dependencies and Linux pthread
target_link_libraries(TwinCatHardware
  pthread
)
ament_target_dependencies(TwinCatHardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface twincat_system.xml)

install(
  TARGETS TwinCatHardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES twincat_system.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()