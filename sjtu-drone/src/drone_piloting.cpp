#include "drone_object_ros.h"
#include <iostream>
#include <geometry_msgs/Pose.h>

unsigned int microsecond = 1000000;

ros::Subscriber pubGtPoseSub;

ros::ServiceClient client;

void PositionCallback(const geometry_msgs::PoseConstPtr& msg) {
   ROS_INFO("Position X: [%If] \n\t\t\t\t Position Y: [%If] \n\t\t\t\t Position Z: [%If] \n", msg->position.x,msg->position.y, msg->position.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_piloting");
  
  ros::NodeHandle node;

  DroneObjectROS drone(node);
  
  drone.velMode(false);
  
  drone.takeOff();
  usleep(1 * microsecond);
  drone.posCtrl(false);
  
  drone.rise(3);
  usleep(1 * microsecond);
  drone.hover();
    
  // generate mesh file
  // client = node.serviceClient<std_srvs::Empty>("/voxblox_node/generate_mesh");
  // if (client.call(srv)) {
  //   ROS_INFO("Success to call service voxblox_node/generate_mesh!");
  // } else {
  //   ROS_ERROR("Failed to call service voxblox_node/generate_mesh");
  //   return 1;
  // }

  // position of drone 
  //pubGtPoseSub = node.subscribe("drone/gt_pose", 1024, PositionCallback);

  ros::spin();

  return 0;
}