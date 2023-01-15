/*

ROS node for horizontal and vertical surface slassification

Author: Andrey Taratin
14/01/23

*/

#include <surface_classifier.h>

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "ros_surface_classifier");

  ROS_INFO("Starting node ros_surface_classifier");

  SurfaceClassifier surf_clas;

  while (ros::ok())
    ros::spin();
}
