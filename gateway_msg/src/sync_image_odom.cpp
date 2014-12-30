#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>


nav_msgs::Odometry odom;

void odometryCallback(const nav_msgs::Odometry &msg)
{
  odom = msg;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << msg->header.stamp << "," 
	    << odom.header.stamp << ", "
	    << odom.pose.pose.position.x << ", "
	    << odom.pose.pose.position.y << ", "
	    << odom.pose.pose.position.z << ", " << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub_cmd = nh.subscribe("odom", 1, odometryCallback);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 10000, imageCallback);
  ros::spin();
}
