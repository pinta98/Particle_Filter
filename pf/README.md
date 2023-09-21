# Particle Filter Localization

This code implement particle filter


## [RTES] How to run this with a bag, and the simulator

1. Run RVIZ2, in another terminal
```
$ rviz2
```
(remember to add at least the 'map' and 'pf->...->Odometry' messages, if you want to see it working)

2. Build and run the node
```
$ colcon build
$ ros2 launch particle_filter demo_particlelaunch.xml
```

3. Play the ROS bag
```
$ ros2 bag play rosbag2_2023_05_19-15_38_30/
```