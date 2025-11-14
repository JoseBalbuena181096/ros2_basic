#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>


typedef pcl::PointXYZRGB PointT;

void cloud_saver(const std::string& file_name, std::string& path, pcl::PointCloud<PointT>::Ptr cloud_arg){
    pcl::PCDWriter cloud_writer;
    cloud_writer.write<PointT>(path + std::string(file_name), *cloud_arg);
}

int main (){
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr pass_cloud_x(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr pass_cloud_y(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr passthrough_cloud (new pcl::PointCloud<PointT> ());
    pcl::PCDReader cloud_reader;
    pcl::PCDWriter cloud_writer;
    
    // Reading the cloud
    std::string path_input = "/home/jose/ros2_ws/src/point_cloud_processing/point_clouds/";
    std::string cloud_name = "tb3_world.pcd";
    std::string output_name = "pass_y_cloud.pcd";
    cloud_reader.read(path_input + cloud_name, *cloud);
    
    // Voxel grid
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    float leaf_size = 0.05f;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    voxel_filter.filter(*passthrough_cloud );

    // Pass through x filter 
    pcl::PassThrough<PointT> passing_x; 
    passing_x.setInputCloud (passthrough_cloud);
    passing_x.setFilterFieldName ("x");
    passing_x.setFilterLimits (-1.5, 1.5);
    passing_x.filter (*passthrough_cloud);

    // Pass through y filter 
    pcl::PassThrough<PointT> passing_y; 
    passing_y.setInputCloud (passthrough_cloud);
    passing_y.setFilterFieldName ("y");
    passing_y.setFilterLimits (-1.5, 1.5);
    passing_y.filter (*passthrough_cloud);

    //**************** Plane segmentation *********/
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr plane_segmented_cloud (new pcl::PointCloud<PointT> ());
    pcl::SACSegmentation<PointT> plane_segmentor;
    pcl::ExtractIndices<PointT> indices_extractor;

    plane_segmentor.setInputCloud(passthrough_cloud);
    plane_segmentor.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentor.setMethodType(pcl::SAC_RANSAC);
    plane_segmentor.setDistanceThreshold(0.01);
    plane_segmentor.segment(*inliers, *coefficients);

    indices_extractor.setInputCloud(passthrough_cloud);
    indices_extractor.setIndices(inliers);
    indices_extractor.setNegative(false);
    indices_extractor.filter(*plane_segmented_cloud);

    // Extract objects (non-plane points)
    pcl::PointCloud<PointT>::Ptr objects_cloud(new pcl::PointCloud<PointT> ());
    indices_extractor.setNegative(true);
    indices_extractor.filter(*objects_cloud);

    //******************************************* Cylinder segmentation */
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    
    pcl::NormalEstimation<PointT, pcl::Normal> normals_estimator;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor;
    pcl::ExtractIndices<PointT> cylinder_indices_extractor;
    pcl::PointCloud<PointT>::Ptr cylinder_cloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<pcl::Normal> cylinder_normal_indices_extractor;
    pcl::PointIndices::Ptr cylinder_in (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr cylinder_co (new pcl::ModelCoefficients);

    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(objects_cloud);
    normals_estimator.setKSearch(30);
    normals_estimator.compute(*cloud_normals);

    cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
    cylinder_segmentor.setDistanceThreshold(0.05);
    cylinder_segmentor.setOptimizeCoefficients(true);
    cylinder_segmentor.setNormalDistanceWeight(0.1);
    cylinder_segmentor.setMaxIterations(10000);

    int loop_counter = 0;

    while(true){
        cylinder_segmentor.setInputCloud(objects_cloud);
        cylinder_segmentor.setInputNormals(cloud_normals);
        cylinder_segmentor.segment(*cylinder_in, *cylinder_co);
        
        if(cylinder_in->indices.empty()){
            break;
        }
        
        cylinder_indices_extractor.setInputCloud(objects_cloud);
        cylinder_indices_extractor.setIndices(cylinder_in);
        cylinder_indices_extractor.setNegative(false);
        cylinder_indices_extractor.filter(*cylinder_cloud);
    
        if(cylinder_cloud->points.size() > 50){
            std::stringstream loop_name_cloud;
            loop_name_cloud << "cylinder_cloud_" << loop_counter << ".pcd";
            cloud_saver(loop_name_cloud.str(), path_input, cylinder_cloud);
            loop_counter++;
        }
            
        // Remove the cylinder from objects_cloud
        cylinder_indices_extractor.setNegative(true);
        cylinder_indices_extractor.filter(*objects_cloud);

        // Remove corresponding normals
        cylinder_normal_indices_extractor.setInputCloud(cloud_normals);
        cylinder_normal_indices_extractor.setIndices(cylinder_in);
        cylinder_normal_indices_extractor.setNegative(true);
        cylinder_normal_indices_extractor.filter(*cloud_normals);
    }
    return 0;
}