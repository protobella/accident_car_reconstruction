cmake_minimum_required(VERSION 3.0.2)
project(voxblox_planner)

find_package(catkin_simple REQUIRED COMPONENTS
  voxblox_ros
)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES voxblox_planner
#  CATKIN_DEPENDS voxblox_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/voxblox_planner.cpp)


 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )


