#ifndef _LOCALIZE_CAM_TAG_H_
#define _LOCALIZE_CAM_TAG_H_

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

//#include "apriltags/TagDetector.h"
//#include "apriltags/Tag36h11.h"
//#include "apriltags/Tag16h5.h"

#include "apriltag.h"
#include "tag36h11.h"
#include "zarray.h"


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class LocalizeCamTag
{
protected:
  cv::Mat img_;
//  geometry_msgs::PoseWithCovarianceStamped pose_; //< Pose buffer  
//  AprilTags::TagDetector *tag_detector_;
//  AprilTags::TagCodes tag_codes_;

  apriltag_detector_t *td_;

  bool vis_;

  float tag_size_;
  float fx_, fy_, cx_, cy_;
  size_t width_, height_;

  ros::Publisher *pub_;
  image_transport::Publisher *pub_image_;

public:
  LocalizeCamTag() 
  : tag_detector_   (NULL)
  //, tag_codes_      (AprilTags::tagCodes36h11)
    // TODO:
  //, tag_codes_      (AprilTags::tagCodes16h5)
  , tag_size_       (0.077)   // Meter
  , fx_             (366.6719)
  , fy_             (367.3712)
  , cx_             (353.4861)
  , cy_             (247.5548)
  , width_          (640)
  , height_         (480)
  , vis_            (false)
  , pub_            (NULL)
  , pub_image_      (NULL)
  , td_             (NULL)
  {
  }
  virtual ~LocalizeCamTag()
  {
    apriltag_detector_destroy(td_);
  }

  void init()
  {
    // TODO:
//    tag_detector_ = new AprilTags::TagDetector(tag_codes_, 1);
//    tag_detector_ = new AprilTags::TagDetector(tag_codes_, 2);
//    cv::namedWindow("view");
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, 1);

  }

  void registerPublisher(ros::Publisher *pub, image_transport::Publisher *pub_image)
  {
    pub_ = pub;
    pub_image_ = pub_image;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    // tf2 broadcaster
    static tf2_ros::TransformBroadcaster br;
    // Covert over cv_bridge
    cv::Mat image_gray;    
    cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::cvtColor(img_ptr->image, img_gray, CV_BGR2GRAY);
    
    // Covnert to zarray
    image_u8_t *img = (image_u8_t *)img_gray.data;
    zarray_t *detections = apriltag_detector_detect(td_, img);
    
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);

      printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n", i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);
                                                                                  apriltag_detection_destroy(det);
                                                                               }
    zarray_destroy(detections);


    
 /*   
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
      
    std::cout << detections.size() << std::endl;
    
 //   geometry_msgs::PoseStamped pose;
 //   pub_->publish(pose);
    

    for (size_t i = 0; i < detections.size(); ++i)
    {
      // TODO:
      if (detections[i].id == 0)
      {

        ROS_INFO("Detected.");
  //      detections[i].draw(img);
      
      // Camera to world
      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      
      detections[i].getRelativeTranslationRotation(tag_size_, fx_, fy_, cx_, cy_, translation, rotation);
      Eigen::Quaterniond q(rotation);
//      	Eigen::Matrix4d tf_w2c = detections[i].getRelativeTransform(tag_size_, fx_, fy_, cx_, cy_);

        geometry_msgs::TransformStamped transformStamped;
  
        transformStamped.header.stamp = ros::Time::now();
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

//        geometry_msgs::PoseStamped pose;
//        std_msgs::Header header;
        
//        pose.pose.position.x = tf_w2c(0, 3);
//        pose.pose.position.y = tf_w2c(1, 3);
//        pose.pose.position.z = tf_w2c(2, 3);
//        pose.pose.orientation.x = 0;
//        pose.pose.orientation.y = 0;
//        pose.pose.orientation.z = 0;
//        pose.pose.orientation.w = 1.0;

//        pose.header.frame_id = "cam";

//	      pub_->publish(pose);
        break; // Only use 1st detection
      }
      
    }  

    if (vis_)
    {
      cv::imshow("view", img_gray_);
      cv::waitKey(1);
    }
//    sensor_msgs::ImagePtr msg_tag = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
//    pub_image_->publish(msg_tag);
 //   detect();
 //   pub_->publish(pose_);
 */
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
  }
 
};

#endif
