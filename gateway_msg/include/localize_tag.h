#ifndef _LOCALIZE_TAG_H_
#define _LOCALIZE_TAG_H_

#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "apriltags/TagDetector.h"
#include "apriltags/Tag36h11.h"
#include "apriltags/Tag16h5.h"

class LocalizeTag
{
protected:
  cv::Mat img_;                                   //< Image buffer
//  geometry_msgs::PoseWithCovarianceStamped pose_; //< Pose buffer  

  AprilTags::TagDetector *tag_detector_;
  AprilTags::TagCodes tag_codes_;

  bool vis_;

  float tag_size_;
  float fx_, fy_, cx_, cy_;
  size_t width_, height_;

  ros::Publisher *pub_;

public:
  LocalizeTag() 
  : tag_detector_   (NULL)
  , tag_codes_      (AprilTags::tagCodes16h5)
  , tag_size_       (0.077)   // Meter
  , fx_             (366.6719)
  , fy_             (367.3712)
  , cx_             (353.4861)
  , cy_             (247.5548)
  , width_          (752)
  , height_         (480)
  , vis_            (true)
  , pub_            (NULL)
  {
  }
  virtual ~LocalizeTag()
  {}

  void init()
  {
    tag_detector_ = new AprilTags::TagDetector(tag_codes_, 1);
    cv::namedWindow("view");
  }

  void registerPublisher(ros::Publisher *pub)
  {
    pub_ = pub;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    // Deep copy to image buffer
    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(img_);
    detect();
 //   pub_->publish(pose_);
  }

//  void publishMessage(ros::Publisher *pub_message)
//  {
//    node_example::node_example_data msg;
//    pub_message->publish(msg);
//  }

//  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
//  {
//    pose_ = *msg;
//  }

  void cmdCallback(const std_msgs::String::ConstPtr &msg)
  {
    ROS_INFO("Receiving cmd %s", msg->data.c_str());
    if (msg->data.empty())
    {
      ROS_WARN("Empty command, skipping...");
      return;
    }
    char cmd = msg->data[0];
    switch(cmd)
    {
      case 'g': // Grab a frame and process
        if (img_.empty())
        {
          ROS_WARN("Empty image, skipping...");
          return;
        }
        detect();
        break;
    }
  }

  void detect()
  {
    cv::Mat img, img_gray;
    
    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.3158, 0.1439, -1.5443e-04, 5.1411e-04, -0.0397);

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    cv::Mat img_undist;

    cv::undistort(img_, img_undist, camera_matrix, dist_coeffs);

    cv::cvtColor(img_undist, img_gray, CV_BGR2GRAY);
    std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(img_gray);
    for (size_t i = 0; i < detections.size(); ++i)
    {
      if (detections[i].id >= 0 && detections[i].id < 8)
      {
      	detections[i].draw(img_gray);
      
//      Eigen::Vector3d translation;
//      Eigen::Matrix3d rotation;
//      detections[i].getRelativeTranslationRotation(tag_size_, fx_, fy_, cx_, cy_, translation, rotation);
  
      	Eigen::Matrix4d tf_w2c = detections[i].getRelativeTransform(tag_size_, fx_, fy_, cx_, cy_);
	geometry_msgs::PoseStamped pose;
        std_msgs::Header header;
        pose.pose.position.x = tf_w2c(0, 3);
        pose.pose.position.y = tf_w2c(1, 3);
        pose.pose.position.z = tf_w2c(2, 3);
      
        pose.header.frame_id = "cam";

	pub_->publish(pose);
      }
      
    }  
    if (vis_)
    {
      cv::imshow("view", img_gray);
      cv::waitKey(1);
    }

  }
 
};

#endif
