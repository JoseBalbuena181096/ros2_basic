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


## K-dimensional Tree Theory
KD TREE :

A k-d tree (short for k-dimensional tree) is a space-partitioning data structure used for organizing points in a k-dimensional space. It is commonly used in computer science, robotics, and computational geometry to answer range and nearest neighbor search queries efficiently.



- Example : 2D Point Cloud

            +---(40, 50)---+
           /               \
    (10, 30)               (70, 60)
   /       \               /       \
(5, 20)   (20, 40)       (55, 75)   (80, 45)
 
In the provided example, we have a 2D point cloud with the following points: (5, 20), (10, 30), (20, 40), (40, 50), (55, 75), (70, 60), and (80, 45). The k-d tree is a data structure that organizes these points in a way that makes range queries and nearest neighbor searches more efficient compared to a linear search through an unorganized point cloud.

The input data is the original point cloud, which is an unordered set of points. The output is a k-d tree that has organized the points hierarchically by partitioning the space along alternating dimensions. In this 2D example, the tree splits the points along the x-axis and y-axis in alternating levels.

The benefits of processing point cloud data using a k-d tree include:

Efficient nearest neighbor search: Instead of comparing the search point to every point in the point cloud, the k-d tree allows for searching only the relevant partitions of the space. This reduces the number of comparisons required and results in faster search times, especially for large point clouds.

Efficient range queries: If you want to find all points within a certain distance of a search point, using a k-d tree can quickly eliminate large portions of the point cloud that are outside of the search range. This leads to faster range queries compared to searching through the entire point cloud.

Spatial partitioning: The k-d tree provides a hierarchical partitioning of the space, which can be useful for other algorithms that require dividing the space into smaller regions (e.g., clustering, segmentation).

However, it's important to note that k-d trees have some limitations:

Inefficiency in high-dimensional spaces: As the number of dimensions increases, the efficiency of k-d trees for searching decreases. This is known as the "curse of dimensionality." In such cases, alternative data structures or algorithms may be more appropriate.

Dynamic updates: Adding, removing, or modifying points in a k-d tree can be computationally expensive, as it may require rebalancing the tree or even reconstructing it from scratch.

To summarize, k-d trees are a powerful data structure for organizing point cloud data, making range queries and nearest neighbor searches more efficient. However, they have limitations in high-dimensional spaces and when dealing with dynamic updates to the point cloud.

