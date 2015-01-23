#include <ros/ros.h>
#include "localize_cam_tag.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizae_cam_tag");
  ros::NodeHandle n;

  LocalizeCamTag *node = new LocalizeCamTag();
  
  node->init();

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_img = it.subscribe("image", 1, &LocalizeCamTag::imageCallback, node);
  image_transport::Publisher pub_img = it.advertise("image_tag", 1);

  ros::Subscriber sub_cmd = n.subscribe("cmd", 1, &LocalizeCamTag::cmdCallback, node);
//  ros::Subscriber sub_pose = n.subscribe("pose", 1, &LocalizeTag::poseCallback, node);
  ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose_tag", 1);
  
  node->registerPublisher(&pub_pose, &pub_img);
  
  ros::Rate r(100);

  while (n.ok())
  {
    node->localize();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
