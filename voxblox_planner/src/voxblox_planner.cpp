#include "voxblox_planner.h"

namespace voxblox_planner {
    VoxbloxPlanning::VoxbloxPlanning(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
                                    : nh_(nh),
                                    nh_private_(nh_private),
                                    voxblox_server_(nh_, nh_private_) {
        odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &VoxbloxPlanning::odometryCallback, this);
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

    void VoxbloxPlanning::odometryCallback(const nav_msgs::Odometry& msg) {
        mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
    }

    double VoxbloxPlanning::getMapDistance(const Eigen::Vector3d& position) const {
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
}  // namespace voxblox_planner