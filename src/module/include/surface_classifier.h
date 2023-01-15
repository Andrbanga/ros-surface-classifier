#ifndef SURFACE_CLASSIFIER
#define SURFACE_CLASSIFIER

// ROS
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

class SurfaceClassifier {

public:
  SurfaceClassifier();

private:
  ros::NodeHandle nh;
  ros::Publisher pc_vert_pub;
  ros::Publisher pc_horiz_pub;
  ros::Subscriber pc_sub;

  int vertical_model_max_iterations; // max itterations of RANSAC vertical plane model
  int horizontal_model_max_iterations; // max itterations of RANSAC horizontal plane model

  double vertical_model_angle_eps;
  double horizontal_model_angle_eps;

  double vertical_model_distance_threshold;
  double horizontal_model_distance_threshold;

  std::string input_pointcloud_topic;

  /**
   * @brief segment vertical and horizontal planes from input PointCloud
   *
   * @param cloud_msg input ros Point cloud
   */
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  /**
   * @brief find all object in input_cloud using seg model and put it to output_cloud
   *
   * @param input_cloud input pcl PointCloud
   * @param output_cloud pcl PointCloud with segmented objects
   * @param seg segmentation model
   */
  void find_objects(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                   pcl::SACSegmentation<pcl::PointXYZ> seg);
};

#endif /* SURFACE_CLASSIFIER */
