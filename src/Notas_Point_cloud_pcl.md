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

