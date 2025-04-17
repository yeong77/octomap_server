# Adaptive voxel mapping based on 3D Occupancy Grid Map using Object Detection

This repository was created based on this [link](https://github.com/OctoMap/octomap_mapping)
The repository is compatible with 'Kinetic' or 'Melodic'.

The sensor used in this project is the RealSense D435. The following repository was referred to for its usage.

[[realsense d435 sdk],](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
[[realsense ros package]](https://github.com/AerialRobotics-IITK/realsense?tab=readme-ov-file)

Additionally, since Octomap does not have a localization function, the localization from [ORB-SLAM2](https://github.com/whatsuppiyush/slam-octomap/tree/master/orb_slam_2_ros) was used.

## Quick start
- Run the RealSense Camera
```
roslaunch realsense2_camera rs_d435_camera_with_model.launch
```
- Run camera-based localization
```
roslaunch orb_slam2_ros orb_slam2_d435_rgbd.launch
```
- Star the octomapping 
```
roslaunch octomap_server octomap_mapping.launch
```
- Give the /resoulution topic for swithcing resolution 
```
rostopic pub -1 /resolution std_msgs/Int32 "data: 2"
```
- Save two octree at once (name1 is low-resolution map, name2 is high-resolution map)
```
rosrun octomap_server octomap_saver -f <name1>.bt <name2>.bt
```

- Load two octree at once (name1 is low-resolution map, name2 is high-resolution map)
```
rosrun octomap_server octomap_merger <name1>.bt <name2>.bt
```

## Result 
![Octomap result](./octomapping.gif)
