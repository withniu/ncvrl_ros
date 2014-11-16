#ifndef _PTAM_METRIC_H_
#define _PTAM_METRIC_H_

#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

class PtamMetric
{
protected:
  cv::Mat img_;                                   //< Image buffer
  geometry_msgs::PoseWithCovarianceStamped pose_; //< Pose buffer  

  AprilTags::TagDetector *tag_detector_;
  AprilTags::TagCodes tag_codes_;

  float tag_size_;
  float fx_, fy_, cx_, cy_;
  size_t width_, height_;
public:
  PtamMetric() 
  : tag_detector_   (NULL)
  , tag_codes_      (AprilTags::tagCodes36h11)
  , tag_size_       (0.166)   // Meter
  , fx_             (585.738365)
  , fy_             (584.053885)
  , cx_             (297.772449)
  , cy_             (231.78081)
  , width_          (640)
  , height_         (480)
  {
    cv::namedWindow("raw");
  }
  virtual ~PtamMetric()
  {}

  void init()
  {
    tag_detector_ = new AprilTags::TagDetector(tag_codes_);
    cv::namedWindow("view");
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    // Deep copy to image buffer
    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(img_);
    cv::imshow("raw", img_);
    cv::waitKey(1);
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    pose_ = *msg;
  }

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
    img_.copyTo(img);
    cv::cvtColor(img, img_gray, CV_BGR2GRAY);
    std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(img_gray);
    for (size_t i = 0; i < detections.size(); ++i)
    {
      detections[i].draw(img);
      
//      Eigen::Vector3d translation;
//      Eigen::Matrix3d rotation;
//      detections[i].getRelativeTranslationRotation(tag_size_, fx_, fy_, cx_, cy_, translation, rotation);
  
      Eigen::Matrix4d m_w2c = detections[i].getRelativeTransform(tag_size_, fx_, fy_, cx_, cy_);
      Eigen::Affine3d tf_w2c;
      tf_w2c.matrix() = m_w2c;
      Eigen::Affine3d tf_c2w = tf_w2c.inverse();
      Eigen::Matrix4d m_c2w = tf_c2w.matrix();

      float z_sfm = pose_.pose.pose.position.z;
      float z_metric = m_c2w(2, 3);
      float scale = z_sfm / z_metric;

      std::cout << "id = " << detections[i].id << std::endl;
//      std::cout << translation << std::endl;
//      std::cout << rotation << std::endl;
      std::cout << tf_c2w.matrix() << std::endl;
      std::cout << "z_sfm = " << z_sfm
                << "z_metric = " << z_metric
                << "scale = " << scale << std::endl;
      
    }  

    cv::imshow("view", img);
    cv::waitKey(1);

  }
 
};

#endif
