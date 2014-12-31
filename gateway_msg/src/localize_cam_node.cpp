#include <ros/ros.h>
#include "localize_cam.h"
#include "image_feature.h"
#include "structure.h"

//#include <glog/logging.h>
int main(int argc, char **argv)
{
//  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "localizae_cam");
  ros::NodeHandle n;

  LocalizeCam *node = new LocalizeCam();
  
  node->init(argv[1], argv[2]);

//  ImageFeature imf;
//  imf.load("sample235/undist/1418932946898065892.jpg.undist.jpg");
//  imf.draw();

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_img = it.subscribe("image_sub", 1, &LocalizeCam::imageCallback, node);
  image_transport::Publisher pub_img = it.advertise("image_pub", 1);

  ros::Subscriber sub_cmd = n.subscribe("cmd", 1, &LocalizeCam::cmdCallback, node);
  ros::Subscriber sub_pose = n.subscribe("pose_sub", 1, &LocalizeCam::poseCallback, node);
  ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 1);
  
  node->registerPublisher(&pub_pose, &pub_img);
  
  ros::Rate r(100);

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
