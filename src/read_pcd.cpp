#include<pcl/io/pcd_io.h>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // load the file

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test.pcd",*cloud)==-1)
    {
        ROS_INFO("Error loadingCouldn't read file test.pcd\n");
        return(-1);
    }
    
 
    /* code */
    return 0;
}