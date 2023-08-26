#include "voxblox_planner.h"

namespace voxblox_planner {
    VoxbloxPlanning::VoxbloxPlanning(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
        : nh_(nh),
        nh_private_(nh_private),
        global_frame_id_("base_link"),
        local_frame_id_("base_link"),
        avoid_collisions_(true),
        voxblox_server_(nh_, nh_private_) {

        constraints_.setParametersFromRos(nh_private_);
        voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);

        nh_private_.param("global_frame_id", global_frame_id_, global_frame_id_);
        nh_private_.param("local_frame_id", local_frame_id_, local_frame_id_);
        nh_private_.param("avoid_collisions", avoid_collisions_, avoid_collisions_);

        // Set up smoothers.
        const double voxel_size = voxblox_server_.getEsdfMapPtr()->voxel_size();

        // Poly smoother.
        poly_smoother_.setParametersFromRos(nh_private_);
        poly_smoother_.setMinCollisionCheckResolution(voxel_size);
        poly_smoother_.setMapDistanceCallback(std::bind(&VoxbloxPlanning::getMapDistance, this, std::placeholders::_1));
        poly_smoother_.setOptimizeTime(true);
        poly_smoother_.setSplitAtCollisions(avoid_collisions_);

        ROS_INFO("Ative VoxbloxPlanning");
    }

    double VoxbloxPlanning::getMapDistance(const Eigen::Vector3d& position) const {
        if (!voxblox_server_.getEsdfMapPtr()) {
            return 0.0;
        }
        double distance = 0.0;
        if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
            return 0.0;
        }
        std_msgs::Float64 distanceFloat;

        distanceFloat.data = distance;

        distCmd.publish(distanceFloat);
        ROS_INFO("Ative getMapDistance");
        return distance;
    }
}  // namespace voxblox_planner