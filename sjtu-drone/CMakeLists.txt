cmake_minimum_required(VERSION 3.2.2)
project(sjtu_drone)

set(CMAKE_CXX_STANDARD 11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    sensor_msgs
    image_transport
    message_generation
    cv_bridge
    tf
    tf2
    tf2_geometry_msgs
    laser_geometry
    gazebo_ros)

find_package(gazebo REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED
	COMPONENTS
	opencv_highgui
  opencv_features2d
	CONFIG
)
find_package(Eigen3 REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

catkin_python_setup()

link_directories(${GAZEBO_LIBRARY_DIRS}
                 ${Boost_LIBRARIES}
                 ${PROTOBUF_LIBRARIES}
                 ${OGRE_LIBRARY_DIRS})

include_directories(${OpenCV_INCLUDE_DIR} 
                    ${EIGEN3_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${OGRE-Terrain_INCLUDE_DIRS}
                    include)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

###################################
catkin_package(
   INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
   LIBRARIES gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
   CATKIN_DEPENDS roscpp rospy image_transport std_msgs sensor_msgs geometry_msgs message_runtime gazebo_ros cv_bridge
   DEPENDS Boost EIGEN3
)

################# 1. A system plugin for gazebo to initialize ROS  #############
add_library( plugin_ros_init SHARED
    src/plugin_ros_init.cpp
)
target_link_libraries( plugin_ros_init 
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${OGRE_LIBRARY_DIRS}
)

set_target_properties( plugin_ros_init
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################## 2. A simple model controller for the quadrotor #############
add_library( plugin_drone SHARED
    src/plugin_drone.cpp
    src/pid_controller.cpp
    include/plugin_drone.h
    include/pid_controller.h
)

target_link_libraries( plugin_drone
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

set_target_properties( plugin_drone
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################## 3. A IMU sensor plugin for the quadrotor ####################
add_library( plugin_ros_imu SHARED
    src/plugin_ros_imu_native.cpp
    include/plugin_ros_imu_native.h
)


target_link_libraries( plugin_ros_imu
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

set_target_properties( plugin_ros_imu
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################# 4. A  sensor plugin for the camera  #########################
add_library( plugin_ros_cam SHARED
    src/plugin_ros_cam.cpp
    src/util_ros_cam.cpp
    include/plugin_ros_cam.h
    include/util_ros_cam.h
)
target_link_libraries( plugin_ros_cam
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${OGRE_LIBRARY_DIRS}
   CameraPlugin
)

set_target_properties( plugin_ros_cam
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################## 7. A sonar sensor plugin for the quadrotor ####################
add_library( plugin_ros_sonar SHARED
    src/plugin_ros_sonar.cpp
    include/plugin_ros_sonar.h
)

target_link_libraries( plugin_ros_sonar
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

set_target_properties( plugin_ros_sonar
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################## 8. A laser sensor plugin for the quadrotor ####################
add_library( plugin_ros_laser SHARED
    src/plugin_ros_laser.cpp
    include/plugin_ros_laser.h
)

target_link_libraries( plugin_ros_laser
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

set_target_properties( plugin_ros_laser
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################## 9. piloting ####################
add_executable( drone_piloting
  src/drone_object_ros.cpp
  src/drone_piloting.cpp
)

target_link_libraries( drone_piloting
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

######### velodyne ###################

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(gazebo_ros_velodyne_gpu_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)

target_compile_definitions(gazebo_ros_velodyne_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)
install(TARGETS gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

