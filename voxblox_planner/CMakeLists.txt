cmake_minimum_required(VERSION 3.0.2)
project(voxblox_planner)

find_package(catkin_simple REQUIRED COMPONENTS
  voxblox_ros
)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/voxblox_planner.cpp
)

add_executable(voxblox_planner_node
 src/voxblox_planner_node.cpp)

target_link_libraries(voxblox_planner_node ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)