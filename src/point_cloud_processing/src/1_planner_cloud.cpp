#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>

int main (){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Add single point to the cloud
    std::string path = "/home/jose/ros2_ws/src/point_cloud_processing/point_clouds/plane.pcd";
    cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud.push_back(pcl::PointXYZ(1.0, 2.0, 0.0));
    cloud.push_back(pcl::PointXYZ(1.0, 0.0, 3.0));
    cloud.push_back(pcl::PointXYZ(0.0, 2.0, 3.0));
    cloud.push_back(pcl::PointXYZ(1.0, 1.0, 2.0));
    cloud.push_back(pcl::PointXYZ(0.5, 1.5, 1.5));
    cloud.push_back(pcl::PointXYZ(2.0, 0.0, 3.0));
    cloud.push_back(pcl::PointXYZ(1.0, 3.0, 1.0));
    cloud.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
    cloud.push_back(pcl::PointXYZ(1.5, 2.5, 2.5));

    pcl::io::savePCDFileASCII(path, cloud);
    std::cout << "Point added to cloud: " << cloud.size() << std::endl;
    
    return 0;
}