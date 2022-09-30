#include<pcl/io/pcd_io.h>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Fill in the cloud data
    cloud.width = 50000;
    cloud.height = 50;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    pcl::io::savePCDFileASCII("test.pcd", cloud);
    /* code */
    return 0;
}
