#ifndef PLUGIN_ROS_LASER_H
#define PLUGIN_ROS_LASER_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
#include <ignition/math/Angle.hh>


namespace gazebo {
class RosLaserPlugin: public SensorPlugin{
public:
    RosLaserPlugin(){topicName = "/velodyne_points";}
    virtual ~RosLaserPlugin(){}

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    virtual void OnNewLaserFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);

protected:
    sensors::GpuRaySensorPtr laser;

    event::ConnectionPtr updated_conn;

    unsigned int width, height/*, depth*/;

    sensor_msgs::LaserScan laser_msg;
    sensor_msgs::PointCloud2 cloud;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;

    ros::NodeHandle* node_handle;
    ros::Publisher pub;
    std::string topicName;
};
}

#endif // PLUGIN_ROS_LASER_H