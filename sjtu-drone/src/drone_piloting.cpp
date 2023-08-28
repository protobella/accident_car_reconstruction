#include "drone_object_ros.h"

#include <iostream>
#include <geometry_msgs/Pose.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

// Subscriber
ros::Subscriber robotpos;
ros::Subscriber down_camera;

// Service
ros::ServiceClient client;

// waypoint variables
geometry_msgs::Point drone_position;
std::vector<cv::Point2f> keypoints_as_waypoints;

// common variables
unsigned int microsecond = 1000000;
int flag = 0;
bool activate_camera = false;

void stageone(DroneObjectROS drone) {
  drone.velMode(false);
  drone.takeOff();
  drone.posCtrl(false);
  drone.rise(1);

  flag =1;
}

void poscallback (const geometry_msgs::Pose::ConstPtr &msg, DroneObjectROS drone) {
  drone_position = msg->position;
  ROS_INFO("x=%f, y=%f, z=%f", drone_position.x, drone_position.y, drone_position.z);
  if (flag == 0) {
    stageone(drone);
  }
  if ((flag ==1 ) && (drone_position.z > 5)) {
    drone.hover();
    flag = 2;
  }
  if (flag == 2) {
    activate_camera = true;
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if (activate_camera) {
    try {
      //activate_camera = false;
      flag = 3;
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      cv::Mat gray_image;
      cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

      cv::GaussianBlur(gray_image, gray_image, cv::Size(5,5), 0);

      cv::Ptr<cv::Feature2D> fdetector = cv::ORB::create();
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      fdetector->detectAndCompute(gray_image, cv::Mat(), keypoints, descriptors);

      cv::Mat image_with_keypoints;
      cv::drawKeypoints(gray_image, keypoints, image_with_keypoints);
      cv::imshow("Manipulated Image With Keypoints", image_with_keypoints);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_piloting");
  
  ros::NodeHandle node;

  DroneObjectROS drone(node);
  
  // position of drone 
  robotpos = node.subscribe<geometry_msgs::Pose>("drone/gt_pose",1, boost::bind(poscallback, _1, drone));
  
  // image callback
  cv::namedWindow("Manipulated Image With Keypoints");
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("/drone/down_camera/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("Manipulated Image With Keypoints");
    
  // generate mesh file
  // client = node.serviceClient<std_srvs::Empty>("/voxblox_node/generate_mesh");
  // if (client.call(srv)) {
  //   ROS_INFO("Success to call service voxblox_node/generate_mesh!");
  // } else {
  //   ROS_ERROR("Failed to call service voxblox_node/generate_mesh");
  //   return 1;
  // }
}