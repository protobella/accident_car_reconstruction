#ifndef VOXBLOX_PLANNER_H_
#define VOXBLOX_PLANNER_H_

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <std_msgs/Float64.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_path_smoothing/polynomial_smoother.h>
#include <mav_planning_common/physical_constraints.h>

namespace voxblox_planner {
  class VoxbloxPlanning {
    public:
      VoxbloxPlanning(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
      virtual ~VoxbloxPlanning() {}
      double getMapDistance(const Eigen::Vector3d& position) const;
      
    private:
      ros::NodeHandle nh_;
      ros::NodeHandle nh_private_;

      ros::NodeHandle n;
      ros::Publisher distCmd = n.advertise<std_msgs::Float64>("/distance_mesh", 10);
     
      // Settings -- frames
      std::string global_frame_id_;
      std::string local_frame_id_;

      // Settings -- constraints.
      mav_planning::PhysicalConstraints constraints_;

      bool avoid_collisions_;

      // Map!
      voxblox::EsdfServer voxblox_server_;

      mav_planning::PolynomialSmoother poly_smoother_;
  };

}  // namespace voxblox_planner
#endif  // VOXBLOX_PLANNER_H_