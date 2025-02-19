# Adaptive voxel mapping based on 3D Occupancy Grid Map using Object 

This repository was created based on this [link](https://github.com/OctoMap/octomap_mapping)
And it was developed based on ROS `Kinetic`. 

Use the __feature/v2-realtime__ branch

## Quick start
- Star the octomapping 
```
roslaunch octomap_server octomap_mapping.launch
```
- Give the /resoulution topic for swithcing resolution 
```
rostopic pub -1 /resolution std_msgs/Float32 "data: 0.5"
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
![Octomap result](./Octomap%20Result.png)
