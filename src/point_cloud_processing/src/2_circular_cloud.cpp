#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>

int main (){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    
    double radius = 1.0;
    int num_points = 100;
    double angular_step = 2.0 * M_PI / num_points;
    for (int i = 0; i < num_points; ++i) {
        pcl::PointXYZRGB point;
        double angle = i * angular_step;
        point.x = radius * std::cos(angle);
        point.y = radius * std::sin(angle);
        point.z = 1.0;
    
        point.r = 255*std::cos(angle);
        point.g = 255*std::sin(angle);
        point.b = 255*std::cos(angle+M_PI_2);
    
        cloud.push_back(point);
    }
    
    
    std::string path = "/home/jose/ros2_ws/src/point_cloud_processing/point_clouds/circular.pcd";
    pcl::io::savePCDFileASCII(path, cloud);
    std::cout << "Point added to cloud: " << cloud.size() << std::endl;
    
    return 0;
}