#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

class Registration
{

private:
  ros::Subscriber sub_lidar_points;
  bool is_first_time = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned;
  double resolution;

public:
  void loop()
  {
    ros::NodeHandle nh;
    nh.param<double>("resolution_value", resolution, 1.0);
    sub_lidar_points = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, &Registration::cloudHandler, this, ros::TransportHints().tcpNoDelay());

    pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    source_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& lasercloudMsg)
  {
    pcl::fromROSMsg(*lasercloudMsg, *pointcloud);
    if (is_first_time)
    {
      *target_cloud = *pointcloud;
      is_first_time = false;
      return;
    }

    *source_cloud = *pointcloud;
    // downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(0.2f, 0.2f, 0.2f);

    voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*downsampled);
    *target_cloud = *downsampled;

    voxelgrid.setInputCloud(source_cloud);
    voxelgrid.filter(*downsampled);
    *source_cloud = *downsampled;

    ros::Time::init();

    std::vector<int> num_threads = {1, omp_get_max_threads()};
    std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
      {"KDTREE", pclomp::KDTREE},
      {"DIRECT7", pclomp::DIRECT7},
      {"DIRECT1", pclomp::DIRECT1}
    };

    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_omp->setResolution(resolution);
    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    aligned = align(ndt_omp, target_cloud, source_cloud);

    *target_cloud = *pointcloud;
  }

  // align point clouds and measure processing time
  pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud ) 
  {
    registration->setInputTarget(target_cloud);
    registration->setInputSource(source_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

    auto t1 = ros::WallTime::now();
    registration->align(*aligned);
    auto t2 = ros::WallTime::now();
    return aligned;
  }
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "liadr_level_test");

  ROS_INFO("\033[1;32m---->\033[0m Program Started.");

  Registration data_align;
  data_align.loop();
    
  ros::spin();
}
