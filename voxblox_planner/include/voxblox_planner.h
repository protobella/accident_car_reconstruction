#ifndef VOXBLOX_PLANNER_H_
#define VOXBLOX_PLANNER_H_

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

namespace voxblox_planner {
  class VoxbloxPlanning {
    public:
      VoxbloxPlanning(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);
      void odometryCallback(const nav_msgs::Odometry& msg);
      virtual ~VoxbloxPlanning() {}
      double getMapDistance(const Eigen::Vector3d& position) const;
    private:
      ros::NodeHandle nh_;
      ros::NodeHandle nh_private_;

      ros::Subscriber odometry_sub_;

      mav_msgs::EigenOdometry odometry_;
      // Map!
      voxblox::EsdfServer voxblox_server_;
  };

}  // namespace voxblox_planner
#endif  // VOXBLOX_PLANNER_H_