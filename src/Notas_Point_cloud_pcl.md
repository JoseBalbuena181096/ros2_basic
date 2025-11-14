## Required Installations
PCL Installations :

```bash
sudo apt install libpcl-dev
sudo apt install pcl-tools
sudo apt-get install ros-humble-pcl-ros
sudo apt-get install ros-humble-pcl-conversions
```
ROS2 Humble : https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

for jazzy:
```bash
sudo apt install libpcl-dev
sudo apt install pcl-tools
sudo apt-get install ros-jazzy-pcl-ros
sudo apt-get install ros-jazzy-pcl-conversions
```
ROS2 Jazzy : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

## Create the package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake point_cloud_processing --dependencies rclcpp sensor_msgs pcl_conversions pcl_ros

ros2 pkg create --build-type ament_cmake point_cloud_processing 
```

## Build the package 

```bash 
cd ~/ros2_ws
colcon build --packages-select point_cloud_processing
source install/setup.bash
```

View the point cloud data 

```bash
cd ~/ros2_ws/src/point_cloud_processing/point_clouds
pcl_viewer table_scene.pcd
```

## Run the node 

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run point_cloud_processing point_cloud_processor
```

## Point cloud library
- Shade Pointers
- Templates classes
- C++

## Visualize multiple point clouds
```bash
pcl_viewer -multiview 2 src/point_cloud_processing/point_clouds/plane.pcd src/point_cloud_processing/point_clouds/plane.pcd 
```


## PCL code Structure Understanding

This quiz tests your understanding of a simple C++ code snippet that uses the Point Cloud Library (PCL) to create a point cloud, add points to it, and save it as an ASCII PCD file. The questions cover various aspects of the code, including the purpose of the code, the number of points added to the point cloud, the library used for point cloud manipulation, the coordinates of specific points in the point cloud, and the function used to save the point cloud to a file. By answering these questions, you demonstrate your comprehension of basic point cloud operations using PCL in C++.

What is the purpose of this code snippet?
To save a point cloud to a PCD file in ASCII format.

How many points are added to the point cloud in the code?

What is the coordinate of the first point in the point cloud?

Which function is used to save the point cloud as an ASCII PCD file?
pcl::io::savePCDFileASCII(path, cloud);

## Filtering and Segmentation
Segmentation RANSAC
If you want to segment planes, spheres, cylinders, etc. from a point cloud, you can use the RANSAC (Random Sample Consensus) algorithm provided by PCL. This is useful for identifying and isolating specific geometric shapes within a point cloud.

What is the purpose of the VoxelGrid filter in the code?

To downsample the point cloud.

What does the PassThrough filter do in the code?

Removes points outside a specified range along the x and y axes.

- What is the primary purpose of the SACSegmentation class in the code?
To segment planar surfaces from the point cloud using the RANSAC algorithm.
- To segment the point cloud into planes and cylinders.

Which search method is used for normal estimation in the code?
- KdTree

Which of the following best describes the overall purpose of the code?
- To segment a point cloud into planes and cylinder and save the segmented objects to files

