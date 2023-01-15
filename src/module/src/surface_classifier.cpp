#include <surface_classifier.h>

SurfaceClassifier::SurfaceClassifier() {

  nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "velodyne_points");

  pc_sub =
      nh.subscribe(input_pointcloud_topic, 1, &SurfaceClassifier::cloud_cb, this);
  pc_vert_pub = nh.advertise<sensor_msgs::PointCloud2>("/vertical_planes", 1);
  pc_horiz_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/horizontal_planes", 1);


  nh.param<int>("vertical_model_max_iterations", vertical_model_max_iterations, 1000);
  nh.param<int>("horizontal_model_max_iterations", horizontal_model_max_iterations, 100);

  nh.param<double>("vertical_model_angle_eps", vertical_model_angle_eps, 1);
  nh.param<double>("horizontal_model_angle_eps", horizontal_model_angle_eps, 1);

  nh.param<double>("vertical_model_distance_threshold", vertical_model_distance_threshold, 0.07);
  nh.param<double>("horizontal_model_distance_threshold", horizontal_model_distance_threshold, 0.07);


}

void SurfaceClassifier::cloud_cb(
    const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

  pcl::PCLPointCloud2 pcl_pc2;

  sensor_msgs::PointCloud2 ros_output_cloud;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_temp_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  // ros to pcl pointcloud2
  pcl_conversions::toPCL(*cloud_msg, pcl_pc2);

  // pcl poitncloud2 to pcl pointcloud
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_temp_cloud);

  // Configure the segmentation object for vertical planes
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle((vertical_model_angle_eps * M_PI) / 180.);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(vertical_model_max_iterations);
  seg.setDistanceThreshold(vertical_model_distance_threshold);

  find_objects(pcl_temp_cloud, pcl_output_cloud, seg);

  // pcl poitncloud to pcl pointcloud2
  pcl::toPCLPointCloud2(*pcl_output_cloud, pcl_pc2);

  // pcl pointcloud2 to ros
  pcl_conversions::moveFromPCL(pcl_pc2, ros_output_cloud);

  ros_output_cloud.header = cloud_msg->header;

  pc_vert_pub.publish(ros_output_cloud);

  // Configure the segmentation object for horizontal planes
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setAxis(Eigen::Vector3f::UnitY());
  seg.setEpsAngle((horizontal_model_angle_eps * M_PI) / 180.);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(horizontal_model_max_iterations);
  seg.setDistanceThreshold(horizontal_model_distance_threshold);

  find_objects(pcl_temp_cloud, pcl_output_cloud, seg);

  // pcl poitncloud to pcl pointcloud2
  pcl::toPCLPointCloud2(*pcl_output_cloud, pcl_pc2);

  // pcl pointcloud2 to ros
  pcl_conversions::moveFromPCL(pcl_pc2, ros_output_cloud);

  ros_output_cloud.header = cloud_msg->header;

  pc_horiz_pub.publish(ros_output_cloud);
}

void SurfaceClassifier::find_objects(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
    pcl::SACSegmentation<pcl::PointXYZ> seg) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment(
      new pcl::PointCloud<pcl::PointXYZ>);

  int original_size = input_cloud->size();

  int plane_counter = 0;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  while ((input_cloud->size() > original_size * 0.3)) {
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      ROS_WARN("Could not estimate a planar model for the given dataset.");
      break;
    }

    plane_counter++;

    // Extract segmented PointCloud from original
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_segment);find_planes

    // ROS_INFO("Found %i planes", plane_counter);
    // ROS_INFO("Plane size: %i", cloud_segment->size());

    // Add segmented points to output PointCloud
    for (int i = 0; i < cloud_segment->size(); i++) {
      pcl::PointXYZ pt = cloud_segment->points[i];
      output_cloud->points.push_back(pt);
    }

    // Erase segmented PointCloud from original
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ> cloudF;
    extract.filter(cloudF);
    input_cloud->swap(cloudF);
  }
}