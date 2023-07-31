#include "plugin_ros_sonar.h"
#include "gazebo/sensors/SonarSensor.hh"
namespace gazebo{
void RosSonarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_INFO("ROS should be initialized first!");
      return;
    }

    if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";
    this->sonar = std::dynamic_pointer_cast<sensors::SonarSensor>(_sensor);

    if (!this->sonar){
        gzerr << "SonarPlugin equires a SonarSensor.\n";
        return;
    }

    ROS_INFO("The Sonar plugin has been loaded!");

    node_handle = new ros::NodeHandle("");
    pub = node_handle->advertise<sensor_msgs::PointCloud2>(topicName, 1, false);
    this->updated_conn = this->sonar->ConnectUpdated(boost::bind(&RosSonarPlugin::onUpdated,this));
}

void RosSonarPlugin::onUpdated(){
    //copy data into ros message
    
    sonar_msg.header.frame_id = "drone_link";
    sonar_msg.header.stamp.sec = this->sonar->LastUpdateTime().sec;
    sonar_msg.header.stamp.nsec = this->sonar->LastUpdateTime().nsec;
    sonar_msg.header.seq = this->sonar->ParentId();

    sonar_msg.angle_increment = 0.523;
    sonar_msg.angle_max = this->sonar->Radius();
    sonar_msg.angle_min = -this->sonar->Radius();
    
    sonar_msg.scan_time = this->sonar->LastMeasurementTime().sec;

    sonar_msg.range_max = this->sonar->RangeMax();
    sonar_msg.range_min = this->sonar->RangeMin();

    uint32_t ranges_size = (sonar_msg.angle_max - sonar_msg.angle_min)/sonar_msg.angle_increment;

    sonar_msg.ranges.assign(ranges_size, this->sonar->Range());
    sonar_msg.intensities.assign(ranges_size, 0);

    projector.transformLaserScanToPointCloud("drone_link", sonar_msg, cloud, listener);

    pub.publish(cloud);
}


GZ_REGISTER_SENSOR_PLUGIN(RosSonarPlugin)
}
