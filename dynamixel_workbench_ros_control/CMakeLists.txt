cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_ros_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # std_msgs
  hardware_interface
  controller_manager
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  # cmake_modules
)

## YAML-CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
  )
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
  )
link_directories(${YAML_CPP_LIBRARY_DIRS})
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

##
set(HEADERS
  include/${PROJECT_NAME}/dynamixel_hardware.h
  )

set(SOURCES
  src/dynamixel_hardware.cpp
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_workbench_ros_control
  CATKIN_DEPENDS roscpp hardware_interface controller_manager dynamixel_workbench_toolbox
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  )

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

add_library(dynamixel_workbench_ros_control ${HEADERS} ${SOURCES})
add_dependencies(dynamixel_workbench_ros_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_ros_control ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(dynamixel_controller_manager_node src/dynamixel_controller_manager_node.cpp)
add_dependencies(dynamixel_controller_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_controller_manager_node dynamixel_workbench_ros_control)
