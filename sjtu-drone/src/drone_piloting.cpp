#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

ros::Publisher pubTakeOff;
ros::Publisher pubCmd;
ros::Publisher pubPosCtrl;
ros::Publisher pubVelMode;

ros::Subscriber pubGtPoseSub;

std_msgs::Bool bool_msg;
geometry_msgs::Twist twist_msg;
std_msgs::Empty empty;

bool isVelMode = false;
bool isPosctrl = false;
bool isFlying = false;

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
  usleep(10 * microsecond);
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

void PositionCallback(const geometry_msgs::PoseConstPtr& msg) {
   ROS_INFO("Position X: [%If] \n\t\t\t\t Position Y: [%If] \n\t\t\t\t Position Z: [%If] \n", msg->position.x,msg->position.y, msg->position.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_piloting");

  ros::NodeHandle node;

  pubTakeOff = node.advertise<std_msgs::Empty>("/drone/takeoff", 1, true);
  pubPosCtrl = node.advertise<std_msgs::Bool>("/drone/posctrl", 1024);
  pubCmd = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  pubVelMode = node.advertise<std_msgs::Bool>("/drone/vel_mode", 1024);

  velMode(false);
  takeOff();
  posCtrl(false);
  moveTo(38,-1.85,2);
  hover();
  
  pubGtPoseSub = node.subscribe("drone/gt_pose", 1024, PositionCallback);

  ros::spin();

  return 0;
}