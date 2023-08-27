#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <image_transport/image_transport.h>
ros::Publisher pubTakeOff;
ros::Publisher pubLand;
ros::Publisher pubReset;
ros::Publisher pubCmd;
ros::Publisher pubPosCtrl;
ros::Publisher pubVelMode;
ros::Subscriber robotpos;
std_msgs::Bool bool_msg;
geometry_msgs::Twist twist_msg;
std_msgs::Empty empty;
geometry_msgs::Point drone_position;
ros::Subscriber down_camera;
std::vector<cv::Point2f> keypoints_as_waypoints;
int flag = 0;
bool isVelMode = false;
bool isPosctrl = false;
bool isFlying = false;
bool activate_camera = false;


unsigned int microsecond = 1000000;

void moveTo(float x, float y, float z) {

  twist_msg.linear.x = x;
  twist_msg.linear.y = y;
  twist_msg.linear.z = z;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = 0;

  pubCmd.publish(twist_msg);
  usleep(1 * microsecond);
  ROS_INFO("Moving to...");
}

void velMode(bool on) {
  isVelMode = on;

  if (isVelMode == true) {
    ROS_INFO("Switching velocity mode on...");
    bool_msg.data = true;
    pubVelMode.publish(bool_msg);
  } else {
    ROS_INFO("Switching velocity mode off...");
  }
}

bool takeOff(void) {
  if (isFlying)
    return false;

  pubTakeOff.publish(empty);
  ROS_INFO("Taking Off...");
  isFlying = true;
  usleep(10 * microsecond);
  return true;
}

bool hover(void) {
  if (!isFlying)
    return false;

  twist_msg.linear.x = 0;
  twist_msg.linear.y = 0;
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  pubCmd.publish(twist_msg);
  ROS_INFO("Hovering...");
  return true;  
}

void posCtrl(bool on) {
  isPosctrl = on;

  bool_msg.data = on ? 1 : 0;
  pubPosCtrl.publish(bool_msg);
  if (on)
    ROS_INFO("Switching position control on...");
  else
    ROS_INFO("Switching position control off...");
}


bool rise(float speed) {
  if (!isFlying)
    return false;

  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = speed;
  twist_msg.angular.x = 0.0; // flag for preventing hovering
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
  pubCmd.publish(twist_msg);
  ROS_INFO("Rising...");
  return true;
}
void stageone() {
  velMode(false);
  takeOff();
  posCtrl(false);
  rise(1);
   
  //moveTo(38,-2,2);
  // hover();
  flag =1;
}
void poscallback (const geometry_msgs::Pose::ConstPtr &msg)
{
  drone_position = msg->position;
  ROS_INFO("x=%f, y=%f, z=%f", drone_position.x, drone_position.y, drone_position.z);
  if (flag == 0){
    stageone();
  }
  if ((flag ==1 ) && (drone_position.z > 5))
  {
    hover();
    flag = 2;
  }
  if (flag == 2)
  {
    activate_camera = true;
  }
}
void imagecallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (activate_camera)
  {
    try {
      activate_camera = false;
      flag = 3;
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // cv::Mat gray_image;
      // cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

      // cv::GaussianBlur(gray_image, gray_image, cv::Size(5,5), 0);

      // cv::Ptr<cv::ORB> orb; 
      // orb->create();
      // std::vector<cv::KeyPoint> keypoints;
      // orb->detect(gray_image, keypoints);

      // for (const cv::KeyPoint& kp : keypoints){
      //   keypoints_as_waypoints.push_back(kp.pt);
      // }
      // cv::Mat image_with_keypoints;
      // cv::drawKeypoints(gray_image, keypoints, image_with_keypoints);

      cv::imshow("Manipulated Image With Keypoints",  cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception %s", e.what());
    }
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_piloting");

  ros::NodeHandle node;
  ros::NodeHandle nh;
  cv::namedWindow("Manipulated Image With Keypoints");

  pubTakeOff = node.advertise<std_msgs::Empty>("/drone/takeoff", 1, true);
  pubLand = node.advertise<std_msgs::Empty>("/drone/land", 1, true);
  pubReset = node.advertise<std_msgs::Empty>("/drone/reset", 1024);
  pubPosCtrl = node.advertise<std_msgs::Bool>("/drone/posctrl", 1024);
  pubCmd = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  pubVelMode = node.advertise<std_msgs::Bool>("/drone/vel_mode", 1024);
  robotpos = node.subscribe<geometry_msgs::Pose>("drone/gt_pose",1, poscallback);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/drone/down_camera/image_raw", 1, imagecallback);

  ros::spin();
  cv::destroyWindow("Manipulated Image With Keypoints");
}