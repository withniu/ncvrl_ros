#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>


nav_msgs::Odometry odom;
tf::TransformListener *listener_ptr;

void odometryCallback(const nav_msgs::Odometry &msg)
{
  odom = msg;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  tf::Stamped<tf::Point> pt, pt_out;
  pt.frame_id_ = "cam";
  pt.stamp_ = ros::Time(0);
  pt.setData(tf::Point(0, 0, 0.1));
  tf::StampedTransform transform;
  try{
    listener_ptr->lookupTransform("/world", "/cam", ros::Time(0), transform);
    listener_ptr->transformPoint("world", pt, pt_out);
  }   
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
//    ros::Duration(1.0).sleep();
    return;
  }   

  tf::Vector3 position = transform.getOrigin();
  std::cout << msg->header.stamp << "," 
	    << transform.stamp_ << ", "
	    << position.x() << ", "
	    << position.y() << ", "
	    << position.z() << ", " 
	    << pt_out.x() << ", "
	    << pt_out.y() << ", "
	    << pt_out.z()  << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  tf::TransformListener listener;
  listener_ptr = &listener;
  ros::Subscriber sub_cmd = nh.subscribe("odom", 1, odometryCallback);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 10000, imageCallback);
  ros::spin();
}
