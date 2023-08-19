#include "voxblox_planner.h"

namespace voxblox_planner {
    VoxbloxPlanning::VoxbloxPlanning(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
                                    : nh_(nh),
                                    nh_private_(nh_private),
                                    verbose_(false),
                                    global_frame_id_("base_link"),
                                    local_frame_id_("base_link"),
                                    voxblox_server_(nh_, nh_private_) {
        // Optionally load a map saved with the save_map service call in voxblox.
        // std::string input_filepath;
        // nh_private_.param("voxblox_path", input_filepath, input_filepath);
        // if (!input_filepath.empty()) {
        //     if (!voxblox_server_.loadMap(input_filepath)) {
        //     ROS_ERROR("Couldn't load ESDF map!");
        //     }
        // }
        double robot_radius = 1.0;
        voxblox_server_.setTraversabilityRadius(robot_radius);
        //voxblox_server_.publishTraversable();

        nh_private_.param("verbose", verbose_, verbose_);
        nh_private_.param("global_frame_id", global_frame_id_, global_frame_id_);
        nh_private_.param("local_frame_id", local_frame_id_, local_frame_id_);

        odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &VoxbloxPlanning::odometryCallback, this);
    
        const double voxel_size = voxblox_server_.getEsdfMapPtr()->voxel_size();
    }

    void VoxbloxPlanning::odometryCallback(const nav_msgs::Odometry& msg) {
        mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
    }

    double VoxbloxPlanning::getMapDistance(const Eigen::Vector3d& position) const {
        double distance = 0.0;
        const bool kInterpolate = false;
        if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, kInterpolate, &distance)) {
            return 0.0;
        }
        ROS_INFO("Distance = %f", distance);
        return distance;
    }
}  // namespace voxblox_planner