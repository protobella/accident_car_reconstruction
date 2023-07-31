#ifndef PLUGIN_ROS_SONAR_H
#define PLUGIN_ROS_SONAR_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>


namespace gazebo {
class RosSonarPlugin: public SensorPlugin{
public:
    RosSonarPlugin(){topicName = "drone/sonar";}
    virtual ~RosSonarPlugin(){}

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    virtual void onUpdated();

protected:
    sensors::SonarSensorPtr sonar;

    event::ConnectionPtr updated_conn;

    sensor_msgs::LaserScan sonar_msg;
    sensor_msgs::PointCloud2 cloud;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;

    ros::NodeHandle* node_handle;
    ros::Publisher pub;
    std::string topicName;
};
}

#endif // PLUGIN_ROS_SONAR_H
