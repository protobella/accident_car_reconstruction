#include "voxblox_planner.h"

ros::Subscriber distSub;
Eigen::Vector3d position(38,-1.85,2);

void DistanceCallback(const std_msgs::Float64::ConstPtr& msg) {
   ROS_INFO("Distance X: [%If] \n", msg->data);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "voxblox_planner");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private;

  FLAGS_alsologtostderr = true;

  voxblox_planner::VoxbloxPlanning node(nh, nh_private);
  ROS_INFO("Initialized Mav Local Planner node.");

  //node.getMapDistance(position);
  distSub = nh.subscribe("/distance_mesh", 1024, DistanceCallback);

  ros::spin();

  return 0;
}