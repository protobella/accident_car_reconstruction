#include "voxblox_planner.h"

double distance = 0.0;
Eigen::Vector3d position(38,-1.85,2);

int main(int argc, char **argv) {
  ros::init(argc, argv, "voxblox_planner");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  voxblox_planner::VoxbloxPlanning node(nh, nh_private);
  ROS_INFO("Initialized Mav Local Planner node.");

  distance = node.getMapDistance(position);
  ROS_INFO("Distance = %f", distance);

  ros::spin();

  return 0;
}