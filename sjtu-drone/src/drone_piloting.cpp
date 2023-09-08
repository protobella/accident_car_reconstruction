#include "drone_object_ros.h"

#include <iostream>
#include <vector>
#include <geometry_msgs/Pose.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <std_srvs/Empty.h>

#include <cmath>

// Subscriber
ros::Subscriber robotpos;
ros::Subscriber down_camera;

// Service
ros::ServiceClient client;
std_srvs::Empty srv;
// waypoint variables
geometry_msgs::Point drone_position;
std::vector<cv::Point2f> keypoints_as_waypoints;
std::vector<cv::Point3d> waypoints;
std::vector<cv::KeyPoint> keypoints;
std::vector<int> idx;
int i = 0;
double rise, pitch, roll;

// fixed waypoints
const int waypoint_size =23;
double waypointx[waypoint_size] = {35.75, 37, 38, 41, 43, 45, 47.5, 47.5, 47.5, 47.5, 47.5, 47.5, 47.5, 47.5, 47.5, 47.5, 47.5, 47.5, 43, 41, 38, 37, 35.75};
double waypointy[waypoint_size] = {-1.85, 0, 1, 2, 2, 1.5, 1, 0, -0.5, -1.2, -2, -2.5, -3, -3.5, -4, -4.5, -5, -5.5, -5.5, -5, -4.5, -4, -3};
double waypointz[waypoint_size] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

// common variables
int flag = 0;
bool activate_camera = false;

DroneObjectROS* drone;

void setDrone(DroneObjectROS& drone_) {
  drone = &drone_;
  ROS_INFO("SetDrone...");
}

void stageone() {
  drone->velMode(false);
  drone->takeOff();
  drone->posCtrl(false);
  drone->rise(0.5);

  flag = 1;
}
double radians(double degree) {
  double pi = 3.14159265359;
  return (degree * (pi / 180));
}

void keypoint_to_coordinate_translate() {
  double height = 480, width= 640;
  double fov = 45;

  double altitude = drone_position.z;
  double fov_radians = radians(fov);

  double scale_x, scale_y;

  scale_x = (2*altitude*tan(fov_radians/2))/width;
  scale_x = (2*altitude*tan(fov_radians/2))/height;
  

  for (const cv::KeyPoint& keypoint : keypoints) {
    double pixel_x = keypoint.pt.x;
    double pixel_y = keypoint.pt.y;

    double real_x = pixel_x * scale_x;
    double real_y = pixel_y * scale_y;

    cv::Point3d waypoint(real_x+drone_position.x, real_y+drone_position.y, 3.0);
    waypoints.push_back(waypoint);
  }
}

void waypoint_move(double x, double y, double z, geometry_msgs::Point dposition) {
  ROS_INFO("Moving to (%f,%f,%f)", x, y, z);

  if (dposition.z >= z) {
    rise = -0.1;
    ROS_INFO("Drone going down");
  } else if (dposition.z < z) {
    rise = 0.1;
    ROS_INFO("Drone going up");
  }
  if (dposition.x >= x) {
    pitch = -0.1;
    ROS_INFO("Drone going back");
  } else if (dposition.x < x) {
    pitch = 0.1;
    ROS_INFO("Drone going front");
  }
  if (dposition.y >= y) {
    roll = -0.1;
    ROS_INFO("Drone going right");
  } else if (dposition.y < y) {
    roll = 0.1;
    ROS_INFO("Drone going left");
  }

  drone->moveTo(pitch, roll, rise);

  if (((x-0.5 < dposition.x) && (dposition.x < x+0.5)) && ((y-0.5 < dposition.y) && (dposition.y < y+0.5)) && ((z-0.5 < dposition.z) && (dposition.z < z+0.5))) {
    drone->hover();

    i++;
  }
}

void poscallback (const geometry_msgs::Pose::ConstPtr &msg) {
  drone_position = msg->position;
  ROS_INFO("x=%f, y=%f, z=%f", drone_position.x, drone_position.y, drone_position.z);
  if (flag == 0) {
    stageone();
  }
  if ((flag ==1) && (drone_position.z > 1)) {
    drone->hover();
    activate_camera = true;
  }
  if (flag == 2) {
    //TODO: function which translates  keypoints_as_waypoints to real world coordinates
    //keypoint_to_coordinate_translate();
    // start movement to waypoints
    if (i < waypoint_size) {
      waypoint_move(waypointx[i], waypointy[i], waypointz[i], drone_position);
    }
    else{
      flag = 3;
    }
    
  }
  // if (flag == 3) {
  //   if (i < (sizeof(waypoints)/sizeof(waypoints[0]))) {
  //     waypoint_move(waypoints[i].x, waypoints[i].y, waypoints[i].z, drone_position);
  //   }
  //   else {
  //     flag = 4;
  //   }
  //   // for (int i = 0; i < keypoints_as_waypoints.size(); i++) {
  //   //   //TODO: get a function that controls drone to move in the direction of the waypoint
  //   //   //https://stackoverflow.com/questions/21613247.5/what-is-the-structure-of-point2f-in-opencv
  //   //   drone->moveTo(keypoints_as_waypoints[i].y, keypoints_as_waypoints[i].x, 1);
  //   //   while (distance_to_waypoint(i)>1);
  //   // }
  // }
  // if (flag == 4) {
  //   //die
  // }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if (activate_camera) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      cv::Mat gray_image;
      cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

      cv::GaussianBlur(gray_image, gray_image, cv::Size(5,5), 0);

      cv::Ptr<cv::Feature2D> fdetector = cv::ORB::create();

      cv::Mat descriptors;
      fdetector->detectAndCompute(gray_image, cv::Mat(), keypoints, descriptors);

      cv::Mat image_with_keypoints;
      cv::drawKeypoints(gray_image, keypoints, image_with_keypoints);
      //cv::KeyPoint::convert(keypoints, keypoints_as_waypoints, idx);
      // cv::imshow("Manipulated Image With Keypoints", image_with_keypoints);
      // cv::waitKey(30);
      flag = 2;
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
  activate_camera = false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_piloting");

  ros::NodeHandle node;

  DroneObjectROS drone(node);

  setDrone(drone);
  
  // position of drone 
  robotpos = node.subscribe<geometry_msgs::Pose>("drone/gt_pose",1,poscallback);
  
  // image callback
  //cv::namedWindow("Manipulated Image With Keypoints");
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub_left = it.subscribe("/drone/down_camera_left/image_raw", 1, imageCallback);
  image_transport::Subscriber sub_right = it.subscribe("/drone/down_camera_right/image_raw", 1, imageCallback);
  
  //cv::destroyWindow("Manipulated Image With Keypoints");
  if (flag == 3){
    // generate mesh file
    client = node.serviceClient<std_srvs::Empty>("/voxblox_node/generate_mesh");
    if (client.call(srv)) {
      ROS_INFO("Success to call service voxblox_node/generate_mesh!");
    } else {
      ROS_ERROR("Failed to call service voxblox_node/generate_mesh");
      return 1;
    }
    flag=4;
  }

  ros::spin();
}