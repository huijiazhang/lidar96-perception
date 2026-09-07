cmake_minimum_required(VERSION 3.0.2)
project(stop_obstacle_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  rrcmc_msgs
)

find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML yaml-cpp)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
    std_msgs
    visualization_msgs
    rrcmc_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

# Patchwork refine node
add_executable(patchwork_refine_node
  src/patchwork_refine_node.cpp
)

target_link_libraries(patchwork_refine_node
  ${catkin_LIBRARIES}
)

# 96-line obstacle detector
# Directly subscribes to /patchwork_refine/no_ground.
# fenced_lane_points_process_cpp is no longer required.
add_executable(points_obstacle_m_96
  src/points_obstacle_m_96.cpp
)

# Wait for rrcmc_msgs message/service headers.
add_dependencies(points_obstacle_m_96
  ${catkin_EXPORTED_TARGETS}
)

if(YAML_FOUND)
  target_compile_definitions(
    points_obstacle_m_96
    PRIVATE HAVE_YAML_CPP=1
  )

  target_include_directories(
    points_obstacle_m_96
    PRIVATE ${YAML_INCLUDE_DIRS}
  )

  target_link_libraries(points_obstacle_m_96
    ${catkin_LIBRARIES}
    ${YAML_LIBRARIES}
  )
else()
  target_link_libraries(points_obstacle_m_96
    ${catkin_LIBRARIES}
  )

  message(WARNING
    "yaml-cpp not found. Building points_obstacle_m_96 without YAML support."
  )
endif()
