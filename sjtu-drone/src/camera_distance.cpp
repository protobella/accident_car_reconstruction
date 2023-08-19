#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

using namespace std;

ros::Subscriber sub;

void distanceCallback(const sensor_msgs::Image::ConstPtr& msg) {

    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg);

    //get depth value at a point
    float distanceVal;

    distanceVal = 0.001*cv_ptr->image.at<u_int16_t>(cv_ptr->image.rows/2, cv_ptr->image.cols);
    std::cout << "Distance value: " << distanceVal << "cm" << endl;

    sub.shutdown();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "camera_distance");

    std::cout << "Getting Image depth value!" << std::endl;

    ros::NodeHandle nh;
    sub = nh.subscribe("/drone/front_camera/image_raw", 1, distanceCallback);
    
    ros::spin();
    
    return 0;
}