#include <ros/ros.h>

#include "ptam_metric.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ptam_metric");
  ros::NodeHandle n;

  PtamMetric *node_ptam_metric = new PtamMetric();
  
  node_ptam_metric->init();

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_img = it.subscribe("image", 1, &PtamMetric::imageCallback, node_ptam_metric);

  ros::Subscriber sub_cmd = n.subscribe("cmd", 1, &PtamMetric::cmdCallback, node_ptam_metric);
  ros::Subscriber sub_pose = n.subscribe("pose", 1, &PtamMetric::poseCallback, node_ptam_metric);

  ros::Rate r(100);

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
