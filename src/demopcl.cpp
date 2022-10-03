#include<pcl/io/pcd_io.h>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>


ros::Publisher pub;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(cloud_filtered);

  // Publish the data
  pub.publish(cloud_filtered);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2>("output", 1);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
  // load the file
  while (ros::ok()) {


    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
      << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
      << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered,
      Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
    pub.publish(cloud_filtered);




    /* code */
  loop_rate.sleep();
  ros::spinOnce();
  }

  ros::spin();
  return 0;
}