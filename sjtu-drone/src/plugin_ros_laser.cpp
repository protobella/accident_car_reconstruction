#include "plugin_ros_laser.h"
#include "gazebo/sensors/GpuRaySensor.hh"
namespace gazebo{
void RosLaserPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    this->laser =std::dynamic_pointer_cast<sensors::GpuRaySensor>(_sensor);

  if (!this->laser)
  {
    gzerr << "GpuRayPlugin not attached to a GpuLaser sensor\n";
    return;
  }

  this->width = this->laser->RangeCount();
  this->height = this->laser->VerticalRangeCount();

  this->updated_conn = this->laser->ConnectNewLaserFrame(
      boost::bind(&RosLaserPlugin::OnNewLaserFrame,
        this, _1, _2, _3, _4, _5));

  this->laser->SetActive(true);
}

void RosLaserPlugin::OnNewLaserFrame(const float * /*_image*/,
    unsigned int /*_width*/, unsigned int /*_height*/,
    unsigned int /*_depth*/, const std::string &/*_format*/){
    //copy data into ros message
    
    laser_msg.header.frame_id = "drone_link";
    laser_msg.header.stamp.sec = this->laser->LastUpdateTime().sec;
    laser_msg.header.stamp.nsec = this->laser->LastUpdateTime().nsec;
    laser_msg.header.seq = this->laser->ParentId();

    laser_msg.angle_increment = this->laser->AngleResolution();
    laser_msg.angle_max = this->laser->AngleMax().Radian();
    laser_msg.angle_min = this->laser->AngleMin().Radian();

    laser_msg.range_max = this->laser->RangeMax();
    laser_msg.range_min = this->laser->RangeMin();

    uint32_t range_count = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;

    laser_msg.ranges.assign(range_count, this->laser->Range(range_count));
    
    laser_msg.intensities.assign(range_count, 0);

    projector.transformLaserScanToPointCloud("drone_link", laser_msg, cloud, listener);

    pub.publish(cloud);
}


GZ_REGISTER_SENSOR_PLUGIN(RosLaserPlugin)
}