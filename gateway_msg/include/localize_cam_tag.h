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
#include <image_geometry/pinhole_camera_model.h>

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

#include <gateway_msg/LocalizeCamTagConfig.h>
#include <dynamic_reconfigure/server.h>

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

  image_geometry::PinholeCameraModel model_;
//  std::unordered_map<int, TagDetection> tag_detections_;

  std::vector<cv::Point3f> object_points_;
  std::vector<cv::Point2f> corners_;

  float tag_size_, offset_x_;
  float fx_, fy_, cx_, cy_;
  size_t width_, height_;

  ros::Publisher *pub_;
  image_transport::Publisher *pub_image_;

public:
  LocalizeCamTag() 
  : tag_size_       (0.165)   // Meter
  , offset_x_       (0.651)
  , fx_             (978.470806)
  , fy_             (981.508278)
  , cx_             (591.695675)
  , cy_             (442.075450)
  , width_          (1280)
  , height_         (960)
  , vis_            (true)
  , pub_            (NULL)
  , pub_image_      (NULL)
  , td_             (NULL)
  , tf_             (NULL)
  { 
    constructObjectPoints();
  }

  void constructObjectPoints() {
    object_points_.clear();
    
    // id 0
    object_points_.push_back(cv::Point3f(0, 0, 0));
    object_points_.push_back(cv::Point3f(tag_size_, 0, 0));
    object_points_.push_back(cv::Point3f(tag_size_, tag_size_, 0));
    object_points_.push_back(cv::Point3f(0, tag_size_, 0));

    // id 1
    cv::Point3f translation(offset_x_, 0, 0); // Offset 0.651m in +x
    object_points_.push_back(object_points_[0] + translation);
    object_points_.push_back(object_points_[1] + translation);
    object_points_.push_back(object_points_[2] + translation);
    object_points_.push_back(object_points_[3] + translation);


    // id 2
    translation = cv::Point3f(offset_x_ * 2, 0, 0); // Offset 2x2x0.651m in +x
    object_points_.push_back(object_points_[0] + translation);
    object_points_.push_back(object_points_[1] + translation);
    object_points_.push_back(object_points_[2] + translation);
    object_points_.push_back(object_points_[3] + translation);
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
  void configCallback(gateway_msg::LocalizeCamTagConfig &config, uint32_t level) 
  {
    vis_ = config.vis;
    ROS_INFO(vis_ ? "Visualization On" : "Visualization Off");
    
    if (offset_x_ != config.offset_x || tag_size_ != config.tag_size)
    {
    
      offset_x_ = config.offset_x;
      tag_size_ = config.tag_size;
      ROS_INFO("Tag size = %f, offset_x = %f", tag_size_, offset_x_);
      constructObjectPoints();
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg,
                     const sensor_msgs::CameraInfoConstPtr &cinfo_msg)
  {
//    ROS_INFO("Callback.");
    static int counter = 0;
     
    model_.fromCameraInfo(cinfo_msg);

    // tf2 broadcaster
    static tf2_ros::TransformBroadcaster br;
    // Covert over cv_bridge
    cv::Mat img_gray, img_roi;    
    cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::cvtColor(img_ptr->image, img_gray, CV_BGR2GRAY);
   
    int width = img_gray.cols;
    int height = img_gray.rows;
    cv::Rect roi(0, 0, width, height);
    if (!corners_.empty() && counter != 0)
    {
      const int b = 50;
      roi = cv::boundingRect(corners_) - cv::Point(b, b) + cv::Size(2 * b, 2 * b);
      // Bound in the image 
      cv::Point tl = roi.tl();
      cv::Point br = roi.br();
      tl.x = tl.x < 0 ? 0 : tl.x;
      tl.y = tl.y < 0 ? 0 : tl.y;
      br.x = br.x > width ? width : br.x;
      br.y = br.y > height ? height : br.y;

      roi = cv::Rect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);
      img_roi = img_gray(roi);
    }
    else
    {
      img_roi = img_gray;
    }

    //std::cout << roi << img_roi.size() << std::endl; 
    corners_.clear();
    std::vector<cv::Point3f> object_points;

    // Covnert to zarray
    // TODO: Avoid hard copy
    image_u8_t *img = image_u8_create(img_roi.cols, img_roi.rows);
    for (int y = 0; y < img->height; ++y)
    {   
      memcpy(&img->buf[y * img->stride], img_roi.ptr(y), sizeof(char) * img->width);
    }

    // Tag detection
    zarray_t *detections = apriltag_detector_detect(td_, img);

 //   if (zarray_size(detections) == 0)
 //     image_u8_write_pnm(img, "/data/tmp.pnm");

    ROS_INFO_THROTTLE(1.0, "# of detection = %d", zarray_size(detections));
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);

//      printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n", i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);
      
      // ID 0,1 is used here
      if (det->id == 0 || det->id == 1 || det->id == 2)
      {
        // Image points
        cv::Point2f offset(roi.x, roi.y);
        
        for (int j = 0; j < 4; ++j)
        {
          corners_.push_back(cv::Point2f(det->p[j][0], det->p[j][1]) + offset);
          object_points.push_back(object_points_[det->id * 4 + j]);
        }
      }
      
      apriltag_detection_destroy(det);

    }
    
    zarray_destroy(detections);
    image_u8_destroy(img);
   
    if (!corners_.empty())
    {
      // PnP
      cv::Mat rvec, tvec;
      //cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
      //cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.339776, 0.111324, -0.000647, 0.001356, 0.000000);
      //cv::Mat camera_matrix = model_.fullIntrinsicMatrix();
      //cv::Mat dist_coeffs = model_.distortionCoeffs();
    
      cv::solvePnP(object_points, corners_, model_.fullIntrinsicMatrix(), model_.distortionCoeffs(), rvec, tvec);
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
    }

    if (vis_)
    {
      cv::cvtColor(img_gray, img_gray, CV_GRAY2BGR);
      cv::rectangle(img_gray, roi, cv::Scalar(0, 0, 255), 3);
      for (int i = 0; i < corners_.size(); ++i)
      {
      //  char buf[100];
      //  sprintf(buf, "%d", i);
      //  cv::putText(img_gray, std::string(buf), corners_[i], cv::FONT_HERSHEY_PLAIN, 20, cv::Scalar(0, 0, 255));
        cv::circle(img_gray, corners_[i], 10, cv::Scalar(0, 0, 255), 5);
      
      }
  
      sensor_msgs::ImagePtr msg_tag = cv_bridge::CvImage(msg->header, "bgr8", img_gray).toImageMsg();
      pub_image_->publish(msg_tag);
 //   detect();
 //   pub_->publish(pose_);
    }
    if (++counter == 100)
      counter = 0;
    
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
