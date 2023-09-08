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
#include <opencv2/calib3d.hpp>
#include <cmath>

// Subscriber
ros::Subscriber robotpos;
ros::Subscriber down_camera;

// Service
ros::ServiceClient client;
 
// waypoint variables
geometry_msgs::Point drone_position;
std::vector<cv::Point2f> keypoints_as_waypoints;
std::vector<cv::Point3d> waypoints;
cv::Mat descriptorsL, descriptorsR;
std::vector<cv::KeyPoint> keypointsL, keypointsR;
std::vector<int> idx;
int i;
int camera_flag[2] = {0,0};
double rise, pitch, roll;


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
  drone->moveTo(0.5,0,1);

  flag = 1;
}

void keypoint_to_coordinate_translate() {
  cv::Mat cameraMatrix = (cv::Mat1d(3, 3, CV_32F) << 343.49636753580074, 0.0,320.5,
                                           0.0, 343.49636753580074,
                                           240.5, 0.0, 0.0, 1.0);

  cv::Mat distCoeffs = (cv::Mat1d(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
  // cv::BFMatcher matcher(cv::NORM_HAMMING);

  // std::vector<cv::DMatch> matches;
  // matcher.match(descriptorsL, descriptorsR, matches);
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));

  std::vector< std::vector<cv::DMatch> > matches;
  ROS_INFO("HERE0");
  matcher->knnMatch( descriptorsL, descriptorsR, matches, 2, cv::noArray() , false);

  ROS_INFO("HERE1");
 //-- Filter matches using the Lowe's ratio test
double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < matches.size(); i++ )
  { double dist = matches[i][0].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< cv::DMatch > good_matches;

  for( int i = 0; i < matches.size(); i++ )
  { if( matches[i][0].distance <= cv::max(2*min_dist, 0.02) )
    { good_matches.push_back( matches[i][0]); }
  }
  ROS_INFO("HERE2");
  cv::Mat essential_matrix;
  cv::Mat mask;
  std::vector<cv::Point2f> ptsL, ptsR;

  for (const cv::KeyPoint& kp :keypointsL){
    ptsL.push_back(kp.pt);
  }
  for (const cv::KeyPoint& kp :keypointsR){
    ptsR.push_back(kp.pt);
  }
  cv::Point2f point;
  point.x=0;point.y=0;
  if (ptsL.size()<ptsR.size())
  {
    while(ptsL.size()<ptsR.size()){
      ptsL.push_back(point);
    }
  }
  else if (ptsL.size()>ptsR.size()) {
    while(ptsL.size()>ptsR.size()){
      ptsL.push_back(point);
    }
  }
  ROS_INFO("%d , %d", ptsL.size(), ptsR.size());

  essential_matrix = cv::findEssentialMat(ptsL, ptsR, cameraMatrix, cv::RANSAC, 0.999, 1.0, mask);

  cv::Mat R, t;
  cv::recoverPose(essential_matrix, ptsL, ptsR, cameraMatrix, R, t, mask);
  ROS_INFO("HERE3");
  // Triangulate 3D points
  cv::Mat1d projMat1 = (cv::Mat1d(3,4, CV_64F) << 1, 0,0,0,
                                                  0, 1,0,0,
                                                  0, 0,1,0);  // Identity matrix

  cv::Mat1d projMat2 = (cv::Mat1d(3,4, CV_64F) << 343.49636753580074, 0.0,320.5, 0,
                                           0.0, 343.49636753580074,
                                           240.5,0, 0.0, 0.0, 1.0, 0);
  std::vector<cv::Point2f> points1, points2;
  for (size_t i = 0; i < good_matches.size(); ++i) {
      if (mask.at<uchar>(i)) {
          points1.push_back(keypointsL[good_matches[i].queryIdx].pt);
          points2.push_back(keypointsR[good_matches[i].trainIdx].pt);
      }
  }
  ROS_INFO("HERE4");
  cv::Mat points4D;
  cv::triangulatePoints(projMat1, projMat2, points1, points2, points4D);

  // Convert 4D homogeneous coordinates to 3D
  for (int i = 0; i < points4D.cols; ++i) {
      cv::Mat point3D = points4D.col(i) / points4D.at<float>(3, i);
      waypoints.push_back(cv::Point3f(point3D.at<float>(0), point3D.at<float>(1), point3D.at<float>(2)));
  }
  ROS_INFO("Points uncovered");
}

void waypoint_move(double x, double y, double z, geometry_msgs::Point dposition) {
  ROS_INFO("Moving to (%f,%f,%f)", x, y, z);

  if (dposition.z >= z) {
    rise = -0.4;
    ROS_INFO("Drone going down");
  } else if (dposition.z < z) {
    rise = 0.4;
    ROS_INFO("Drone going up");
  }
  if (dposition.x >= x) {
    pitch = -0.4;
    ROS_INFO("Drone going back");
  } else if (dposition.x < x) {
    pitch = 0.4;
    ROS_INFO("Drone going front");
  }
  if (dposition.y >= y) {
    roll = -0.4;
    ROS_INFO("Drone going right");
  } else if (dposition.y < y) {
    roll = 0.4;
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
  if ((flag ==1) && (drone_position.z > 10)) {
    drone->hover();
    activate_camera = true;
  }
  if ((flag == 2) && (camera_flag[0]==1) && (camera_flag[1]=1)) {
    // activate_camera = false;
    //TODO: function which translates  keypoints_as_waypoints to real world coordinates
    keypoint_to_coordinate_translate();
    // start movement to waypoints
    flag = 3;
  }
  if (flag == 3) {
    if (i < (sizeof(waypoints)/sizeof(waypoints[0]))) {
      waypoint_move(waypoints[i].x, waypoints[i].y, waypoints[i].z, drone_position);
    }
    else {
      flag = 4;
    }
    // for (int i = 0; i < keypoints_as_waypoints.size(); i++) {
    //   //TODO: get a function that controls drone to move in the direction of the waypoint
    //   //https://stackoverflow.com/questions/21613246/what-is-the-structure-of-point2f-in-opencv
    //   drone->moveTo(keypoints_as_waypoints[i].y, keypoints_as_waypoints[i].x, 1);
    //   while (distance_to_waypoint(i)>1);
    // }
  }
  if (flag == 4) {
    //die
  }
}

void imageCallback1(const sensor_msgs::ImageConstPtr& msg) {
  if (activate_camera) {
    try {
      if (camera_flag[0]==0) {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      cv::Mat gray_image;
      cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

      cv::GaussianBlur(gray_image, gray_image, cv::Size(5,5), 0);

      cv::Ptr<cv::Feature2D> fdetector = cv::ORB::create();

      fdetector->detectAndCompute(gray_image, cv::Mat(), keypointsL, descriptorsL);
      cv::imwrite("/home/alphad/catkin_ws/CroquiL.jpg", cv_ptr->image);      
      cv::Mat image_with_keypoints;
      cv::drawKeypoints(gray_image, keypointsL, image_with_keypoints);
      //cv::KeyPoint::convert(keypointsL, keypoints_as_waypoints, idx);
      cv::imwrite("/home/alphad/catkin_ws/CroquiL_key.jpg", image_with_keypoints);
      // cv::imshow("Manipulated Image With KeypointsL", image_with_keypoints);
      // cv::waitKey(30);
      flag = 2;
      camera_flag[0]=1;
      }
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

}
void imageCallback2(const sensor_msgs::ImageConstPtr& msg) {
  if (activate_camera) {
    try {
      if (camera_flag[1]==0) {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      cv::Mat gray_image;
      cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

      cv::GaussianBlur(gray_image, gray_image, cv::Size(5,5), 0);

      cv::Ptr<cv::Feature2D> fdetector = cv::ORB::create();

      cv::Mat descriptors;
      fdetector->detectAndCompute(gray_image, cv::Mat(), keypointsR, descriptorsR);
      cv::imwrite("/home/alphad/catkin_ws/CroquiR.jpg", cv_ptr->image);
      cv::Mat image_with_keypoints;
      cv::drawKeypoints(gray_image, keypointsR, image_with_keypoints);
      //cv::KeyPoint::convert(keypointsR, keypoints_as_waypoints, idx);
      cv::imwrite("/home/alphad/catkin_ws/CroquiR_key.jpg", image_with_keypoints);
      // cv::imshow("Manipulated Image With KeypointsR", image_with_keypoints);
      // cv::waitKey(30);
      flag = 2;
      camera_flag[1]=1; 
      }
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

  setDrone(drone);
  
  // position of drone 
  robotpos = node.subscribe<geometry_msgs::Pose>("drone/gt_pose",1,poscallback);
  
  // image callback
  //cv::namedWindow("Manipulated Image With KeypointsL");
  //cv::namedWindow("Manipulated Image With KeypointsR");
  image_transport::ImageTransport itL(node);
  image_transport::ImageTransport itR(node);
  image_transport::Subscriber left = itL.subscribe("/drone/down_camera_left/image_raw", 1, imageCallback1);
  image_transport::Subscriber right = itR.subscribe("/drone/down_camera_right/image_raw", 1, imageCallback2);
  ros::spin();
  //cv::destroyWindow("Manipulated Image With KeypointsL");
  //cv::destroyWindow("Manipulated Image With KeypointsR");
  // generate mesh file
  // client = node.serviceClient<std_srvs::Empty>("/voxblox_node/generate_mesh");
  // if (client.call(srv)) {
  //   ROS_INFO("Success to call service voxblox_node/generate_mesh!");
  // } else {
  //   ROS_ERROR("Failed to call service voxblox_node/generate_mesh");
  //   return 1;
  // }
}