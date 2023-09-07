#include "voxblox_planner.hpp"

YourPlannerVoxblox::YourPlannerVoxblox(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      voxblox_server_(nh_, nh_private_) {
  // Optionally load a map saved with the save_map service call in voxblox.
  std::string input_filepath;
  nh_private_.param("voxblox_path", input_filepath, input_filepath);
  if (!input_filepath.empty()) {
    if (!voxblox_server_.loadMap(input_filepath)) {
      ROS_ERROR("Couldn't load ESDF map!");
    }
  }
  double robot_radius = 1.0;
  voxblox_server_.setTraversabilityRadius(robot_radius);
  voxblox_server_.publishTraversable();
}

double YourPlannerVoxblox::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    return 0.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    return 0.0;
  }
  return distance;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"abc");

    ros::spin();
}
