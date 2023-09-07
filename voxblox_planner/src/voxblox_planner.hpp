#include "voxblox_ros/esdf_server.h"
#include "ros/ros.h"
#include "voxblox_ros/ros_params.h"
class YourPlannerVoxblox {
 public:
  YourPlannerVoxblox(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  virtual ~YourPlannerVoxblox() {}
  double getMapDistance(const Eigen::Vector3d& position) const;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Map!
  voxblox::EsdfServer voxblox_server_;
};