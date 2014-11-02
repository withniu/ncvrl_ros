#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

cv::Mat img_draw;
cv::Mat img; 
cv::Mat img_cam;

void callback_img(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(img_cam);
}


std::list<cv::Point2f> tail, tail_sfm;

Eigen::Vector3f t_sfm;
Eigen::Quaternionf q_sfm;


Eigen::Vector2f projectToGround(const Eigen::Vector3f &p, const Eigen::Quaternionf &q, const Eigen::Vector3f &t)
{
  
}

void draw(const Eigen::Quaternionf &q, const Eigen::Vector3f &translation, const cv::Scalar &color, std::list<cv::Point2f> &tail, bool draw_tail = true)
{

  // Project z axis to ground plane
  Eigen::Vector3f z(0.0, 0.0, 1.0);
  z = q.matrix() * z + translation;  

  // Metric dimension in 3D coordinate frame
  float d_x = 15.24;        // In meter
  float d_y = 28.6512;      // In meter
  
  // Convert to image coordinate  
  float s_x = img_draw.cols;
  float s_y = img_draw.rows;

  float o_x = s_x - translation(1) / d_y * s_x;
  float o_y = s_y - translation(0) / d_x * s_y;

  float p_x = s_x - z(1) / d_y * s_x;
  float p_y = s_y - z(0) / d_x * s_y;

  float s = sqrt((p_x - o_x) * (p_x - o_x) + (p_y - o_y) * (p_y - o_y));

  float u = (p_x - o_x) / s;
  float v = (p_y - o_y) / s;

  float theta = atan2(v, u);

  // 2D rotation matrix and translation
  Eigen::Rotation2Df rot(theta);
  Eigen::Vector2f t(o_x, o_y);

  // Camera cone
  std::vector<Eigen::Vector2f> camera;
  float l = 5;
  Eigen::Vector2f o = rot * Eigen::Vector2f(0.0, 0.0) + t;
  Eigen::Vector2f a = rot * Eigen::Vector2f(3 * l, -3 * l) + t;
  Eigen::Vector2f b = rot * Eigen::Vector2f(3 * l, 3 * l) + t;

  if (tail.size() == 250)
    tail.pop_front();

  tail.push_back(cv::Point2f(o(0), o(1)));

  cv::circle(img_draw, cv::Point2f(o_x, o_y), 3, color);
  cv::line(img_draw, cv::Point2f(o(0), o(1)), cv::Point2f(a(0), a(1)), color, 2, CV_AA);
  cv::line(img_draw, cv::Point2f(o(0), o(1)), cv::Point2f(b(0), b(1)), color, 2, CV_AA);
  cv::line(img_draw, cv::Point2f(a(0), a(1)), cv::Point2f(b(0), b(1)), color, 2, CV_AA);

  if (true == draw_tail)
  {
   cv::Point2f p1, p2;
   for (std::list<cv::Point2f>::iterator iter = tail.begin(); iter != tail.end(); ++iter)
   {
      if (iter == tail.begin())
      {
        p1 = *iter;
       continue;
     }
      p2 = *iter;  
      cv::line(img_draw, p1, p2, color, 2, CV_AA);
     p1 = p2;
  }
  }
}


void callback_sfm(const geometry_msgs::PoseStamped::ConstPtr &pose_sfm)
{
  
  Eigen::Vector3f translation(pose_sfm->pose.position.x, 
                              pose_sfm->pose.position.y, 
                              pose_sfm->pose.position.z);
  Eigen::Quaternionf q(pose_sfm->pose.orientation.w, 
                       pose_sfm->pose.orientation.x, 
                       pose_sfm->pose.orientation.y, 
                       pose_sfm->pose.orientation.z);

  t_sfm = translation;
  q_sfm = q;

}
void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msf)//,
//                    const geometry_msgs::PoseStamped::ConstPtr &pose_sfm)
{

  // Extract translation and rotation
  Eigen::Vector3f translation(pose_msf->pose.pose.position.x, 
                              pose_msf->pose.pose.position.y, 
                              pose_msf->pose.pose.position.z);
  Eigen::Quaternionf q(pose_msf->pose.pose.orientation.w, 
                       pose_msf->pose.pose.orientation.x, 
                       pose_msf->pose.pose.orientation.y, 
                       pose_msf->pose.pose.orientation.z);
  
  img.copyTo(img_draw);
  draw(q, translation, cv::Scalar(255, 0, 0), tail); 
   
  draw(q_sfm, t_sfm, cv::Scalar(200, 200, 200), tail_sfm, false);
/*
  int c = img_cam.cols;
  int r = img_cam.rows;
  int cd = img_draw.cols;
  int rd = img_draw.rows;

  cv::Mat img_cam_r;
  cv::resize(img_cam, img_cam_r, cv::Size(rd * c / r, rd));

//  std::cout << img_cam_r.size() << std::endl;
//  std::cout << img_draw.size() << std::endl;

  cv::Mat img_sbs(img_draw.rows, img_draw.cols + img_cam_r.cols, CV_32FC3);
//  img_draw.convertTo(img_draw, CV_32FC3);
//  img_cam_r.convertTo(img_cam_r, CV_32FC3);

  for (size_t c = 0; c < img_draw.cols; ++c)
  {
    img_sbs.col(c) = img_draw.col(c);
  }


  for (size_t c = 0; c < img_cam_r.cols; ++c)
  {
    img_sbs.col(c + img_draw.cols) = img_cam_r.col(c);
  }
*/
  cv::resize(img_draw, img_draw, cv::Size(), 0.7, 0.7);

 // cv::namedWindow("2d-1");
  cv::namedWindow("2d-2");
 // cv::imshow("2d-1", img_cam_r);
  cv::imshow("2d-2", img_draw);

  cv::waitKey(1);
}


int main(int argc, char **argv)
{

  img = cv::imread("/home/withniu/court.png");
  img.copyTo(img_draw);


  ros::init(argc, argv, "viewer2d");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("cam0", 1, callback_img);
//  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> msf_sub(nh, "viewer/msf", 1);
//  message_filters::Subscriber<geometry_msgs::PoseStamped> sfm_sub(nh, "viewer/sfm", 1);
//  message_filters::TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseStamped> sync(msf_sub, sfm_sub, 1000);
//  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::Subscriber sub = nh.subscribe("viewer/pose", 1, callback);
  ros::Subscriber sub_sfm = nh.subscribe("viewer/pose_sfm", 1, callback_sfm);
  ros::spin();

  return 0;
}
