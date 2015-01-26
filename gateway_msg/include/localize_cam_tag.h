#ifndef _LOCALIZE_CAM_TAG_H_
#define _LOCALIZE_CAM_TAG_H_

#include <iostream>
//#include <unordered_map>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "apriltag_c/apriltag.h"
#include "apriltag_c/tag36h11.h"
#include "apriltag_c/common/zarray.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


struct TagDetection
{
  int id;
  std::vector<cv::Point2f> corners;
};


class LocalizeCamTag
{
protected:
  cv::Mat img_;
//  geometry_msgs::PoseWithCovarianceStamped pose_; //< Pose buffer  

  apriltag_detector_t *td_;
  apriltag_family_t *tf_;
  bool vis_;

//  std::unordered_map<int, TagDetection> tag_detections_;

  std::vector<cv::Point3f> object_points_;
  std::vector<cv::Point2f> corners_;

  float tag_size_;
  float fx_, fy_, cx_, cy_;
  size_t width_, height_;

  ros::Publisher *pub_;
  image_transport::Publisher *pub_image_;

public:
  LocalizeCamTag() 
  : tag_size_       (0.165)   // Meter
  , fx_             (978.470806)
  , fy_             (981.508278)
  , cx_             (591.695675)
  , cy_             (442.075450)
  , width_          (1280)
  , height_         (960)
  , vis_            (false)
  , pub_            (NULL)
  , pub_image_      (NULL)
  , td_             (NULL)
  , tf_             (NULL)
  {
    object_points_.push_back(cv::Point3f(0, 0, 0));
    object_points_.push_back(cv::Point3f(tag_size_, 0, 0));
    object_points_.push_back(cv::Point3f(tag_size_, tag_size_, 0));
    object_points_.push_back(cv::Point3f(0, tag_size_, 0));
  }
  
  
  virtual ~LocalizeCamTag()
  {
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

  void init()
  {
    // TODO:
    // Tag detection params
    td_ = apriltag_detector_create();
    tf_ = tag36h11_create();
    tf_->black_border = 1;
    apriltag_detector_add_family(td_, tf_);
    td_->quad_decimate = 1.0;
    td_->quad_sigma = 0.0;
    td_->nthreads = 4;
    td_->debug = false;
    td_->refine_decode = 0;
    td_->refine_pose = 0;

  }

  void registerPublisher(ros::Publisher *pub, image_transport::Publisher *pub_image)
  {
    pub_ = pub;
    pub_image_ = pub_image;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
//    ROS_INFO("Callback.");
    
    // tf2 broadcaster
    static tf2_ros::TransformBroadcaster br;
    // Covert over cv_bridge
    cv::Mat img_gray, img_roi;    
    cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::cvtColor(img_ptr->image, img_gray, CV_BGR2GRAY);
    
    cv::Rect roi(0, 0, img_gray.cols, img_gray.rows);
    if (!corners_.empty())
    {
      roi = cv::boundingRect(corners_) + cv::Size(30, 30);
      img_roi = img_gray(roi);
    }
    else
    {
      img_roi = img_gray;
    }

    // Covnert to zarray
    // TODO: Avoid hard copy
    image_u8_t *img = image_u8_create(img_roi.cols, img_roi.rows);
    for (int y = 0; y < img->height; ++y)
    {   
      memcpy(&img->buf[y * img->stride], img_roi.ptr(y), sizeof(char) * img->width);
    }
//    image_u8_write_pnm(img, "/home/withniu/tmp.pnm");

    // Tag detection
    zarray_t *detections = apriltag_detector_detect(td_, img);
    
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);

      printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n", i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);
      
      // ID 0 is used here
      if (det->id == 0)
      {
        corners_.clear();
        // Image points
        cv::Point2f offset(roi.x, roy.y);
        corners_.push_back(cv::Point2f(det->p[0][0], det->p[0][1]) + offset);
        corners_.push_back(cv::Point2f(det->p[1][0], det->p[1][1]) + offset);
        corners_.push_back(cv::Point2f(det->p[2][0], det->p[2][1]) + offset);
        corners_.push_back(cv::Point2f(det->p[3][0], det->p[3][1]) + offset);
      	// PnP
        cv::Mat rvec, tvec;
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
        cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.339776, 0.111324, -0.000647, 0.001356, 0.000000);
        cv::solvePnP(object_points_, corners_, camera_matrix, dist_coeffs, rvec, tvec);
        // Convert to cam to world
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        R = R.t();
        tvec = -R * tvec;

	      // Convert to Eigen
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        
        translation << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
        rotation << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), 
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), 
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
        Eigen::Quaterniond q(rotation);
        
        // Wrap as tf2
        geometry_msgs::TransformStamped transformStamped;
  
        transformStamped.header.stamp = msg->header.stamp;  // Use image stamp
        transformStamped.header.frame_id = "marker_origin";
        transformStamped.child_frame_id = "camera";
        
        transformStamped.transform.translation.x = translation.x();
        transformStamped.transform.translation.y = translation.y();
        transformStamped.transform.translation.z = translation.z();

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
      
        ROS_INFO("%f, %f, %f", translation.x(), translation.y(), translation.z());
      }
      apriltag_detection_destroy(det);

    }
    zarray_destroy(detections);
    image_u8_destroy(img);
    
    if (vis_)
    {
//    sensor_msgs::ImagePtr msg_tag = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
//    pub_image_->publish(msg_tag);
 //   detect();
 //   pub_->publish(pose_);
    }
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
        break;
    }
  }

};

#endif
