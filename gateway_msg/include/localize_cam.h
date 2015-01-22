#ifndef _LOCALIZE_CAM_H_
#define _LOCALIZE_CAM_H_

#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/flann/flann.hpp>

#include "SiftGPU.h"
#include "gl.h"
#include <Eigen/Dense>

#include "structure.h"

class LocalizeCam
{
protected:
  cv::Mat img_;                                   //< Image buffer
//  geometry_msgs::PoseWithCovarianceStamped pose_; //< Pose buffer 
  geometry_msgs::PoseStamped pose_;
  SiftGPU sift_;

  Structure st_;
  cv::Mat rvec_, tvec_;

  cv::FlannBasedMatcher matcher_;
  bool vis_;

  float fx_, fy_, cx_, cy_;
  size_t width_, height_;

  tf::TransformListener listener_;  

  ros::Publisher *pub_;
  image_transport::Publisher *pub_image_;

public:
  LocalizeCam() 
  : fx_             (366.6719)
  , fy_             (367.3712)
  , cx_             (353.4861)
  , cy_             (247.5548)
  , width_          (752)
  , height_         (480)
  , vis_            (false)
  , pub_            (NULL)
  , pub_image_      (NULL)
  {
    //char * argv[] = {"-fo", "-1", "-tc2", "7680", "-da"};
    char * argv[] = {"-fo", "-1", "-tc2", "512", "-da"};
    sift_.ParseParam(5, argv);
    int support = sift_.CreateContextGL();
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
      ROS_ERROR("SiftGPU failed.");
      return;
    }

  }


  virtual ~LocalizeCam()
  {
    st_.freeDatabase();
  }

  void init(const std::string &filename_nvm, const std::string &filename_cfg)
  {
    //cv::namedWindow("view");
    st_.load(filename_nvm, filename_cfg);
    st_.buildDatabase();
    st_.buildFlannIndex();
  }

  void registerPublisher(ros::Publisher *pub, image_transport::Publisher *pub_image)
  {
    pub_ = pub;
    pub_image_ = pub_image;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv::Mat img, img_gray;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);

// Undistort image
//    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.3158, 0.1439, -1.5443e-04, 5.1411e-04, -0.0397);
//    cv::Mat img_undist;
//    cv::undistort(img_, img_undist, camera_matrix, dist_coeffs);

    cv::Mat img_msg = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor(img_msg, img_gray, CV_BGR2GRAY);
    cv::cvtColor(img_gray, img, CV_GRAY2BGR);
      
    sift_.RunSIFT(img_gray.cols, img_gray.rows, img_gray.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);

    int num = sift_.GetFeatureNum();
    std::vector<float> descriptors(128 * num);
    std::vector<SiftGPU::SiftKeypoint> keys(num);
    sift_.GetFeatureVector(&keys[0], &descriptors[0]);
    
    // Convert ot OpenCV
    std::vector<cv::KeyPoint> keypoints(keys.size());
    for (size_t i = 0; i < keys.size(); ++i)
    {
      SiftGPU::SiftKeypoint key = keys[i];
      // TODO: check the minus sign for orientation
      cv::KeyPoint kp(key.x, key.y, key.s, -key.o / 3.1416 * 180);
      keypoints[i] = kp;
    }

//    cv::drawKeypoints(img, keypoints, img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::Mat query_desc(num, 128, CV_32F, &descriptors[0]);
    query_desc *= 512.0f;
/*
    cv::Mat query_desc(num, 128, CV_32F);
    for (int r = 0; r < num; ++r)
      for (int c = 0; c < 128; ++c)
      {
        query_desc.at<float>(r, c) = descriptors[r * 128 + c] * 512.0f;
      
      }
*/
#ifdef DEBUG_MATCHING
    cv::Mat qd_img;
    query_desc.convertTo(qd_img, CV_8U, 512.0);
    cv::imwrite("qerry_img.png", qd_img);
    cv::FileStorage fs("qerry_img.yml", cv::FileStorage::WRITE);
    fs << "querry" << qd_img;
    fs.release();

    tf::StampedTransform transform;
    try{
      listener_.lookupTransform("/cam", "/world", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }


    tf::Matrix3x3 rot_gt = transform.getBasis();
    tf::Vector3 t_gt = transform.getOrigin();
    cv::Mat tvec_gt = (cv::Mat_<double>(3, 1) << t_gt.x(), t_gt.y(), t_gt.z());
    tf::Vector3 r0 = rot_gt[0];
    tf::Vector3 r1 = rot_gt[1];
    tf::Vector3 r2 = rot_gt[2];

    cv::Mat rot_mat_gt = (cv::Mat_<double>(3, 3) << r0.x(), r0.y(), r0.z(),
                                                    r1.x(), r1.y(), r1.z(),
                                                    r2.x(), r2.y(), r2.z());
    cv::Mat rvec_gt;
    cv::Rodrigues(rot_mat_gt, rvec_gt);
#endif
    // Localize camera pose
    st_.localize(img, query_desc, keypoints, camera_matrix, rvec_, tvec_);
    
    cv::Mat rvec, tvec;
    rvec = rvec_;
    tvec = tvec_;
    cv::Mat R, tf;
    cv::Rodrigues(rvec, R);
    hconcat(R, tvec, tf);
    cv::Mat xyz = - R.t() * tvec;

    cv::Mat Rinv = R.t();
    Eigen::Matrix3f rot;
    rot << Rinv.at<double>(0, 0), Rinv.at<double>(0, 1), Rinv.at<double>(0, 2),
           Rinv.at<double>(1, 0), Rinv.at<double>(1, 1), Rinv.at<double>(1, 2),
           Rinv.at<double>(2, 0), Rinv.at<double>(2, 1), Rinv.at<double>(2, 2);

    Eigen::Quaternionf q(rot);

//    std::cout << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
//    std::cout << query_desc.row(0) << std::endl;
//    cv::waitKey(0);

 /* 
      	Eigen::Matrix4d tf_w2c = detections[i].getRelativeTransform(tag_size_, fx_, fy_, cx_, cy_);
*/	      
     geometry_msgs::PoseStamped pose;
     std_msgs::Header header;
     pose.header = msg->header;
     pose.pose.position.x = xyz.at<double>(0);
     pose.pose.position.y = xyz.at<double>(1);
     pose.pose.position.z = xyz.at<double>(2);
     pose.pose.orientation.x = q.x();
     pose.pose.orientation.y = q.y();
     pose.pose.orientation.z = q.z();
     pose.pose.orientation.w = q.w();

     pose.header.frame_id = "world";

     pub_->publish(pose);

     if (vis_)
     {
       cv::imshow("view", img_gray);
       cv::waitKey(1);
     }
     std_msgs::Header header_img = msg->header;
//    header_img.frame_id = "cam_sfm";
     sensor_msgs::ImagePtr msg_tag = cv_bridge::CvImage(header_img, "bgr8", img).toImageMsg();
     pub_image_->publish(msg_tag);
}

//  void publishMessage(ros::Publisher *pub_message)
//  {
//    node_example::node_example_data msg;
//    pub_message->publish(msg);
//  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
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
  }
 
};

#endif
