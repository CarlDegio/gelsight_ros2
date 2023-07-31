# GelSight_ROS2
## introduction
ROS2 humble interface for GelSight MINI tactile sensor.

## requirements
- ROS2 humble
- GelSight SDK install on python([gsrobotics](https://github.com/gelsightinc/gsrobotics))

## usage
1. clone this repository to your ros2 workspace
2. build your ros2 workspace
3. run `ros2 run gelsight_ros2 gelsight_showimage` to get raw rgb image on topic `/gsmini/image`
4. run `ros2 run gelsight_ros2 gelsight_show3d` to get 3d point cloud on topic `/gsmini/pcd`

## known issues

3d point cloud show in rviz2 is strange

gelsight mini crush sometimes