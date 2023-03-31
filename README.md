# Benchmarking Vision based Grasp Detection algorithms

Research under Prof Berk Calli (Manipulation and Environmental Robotics Lab, Worcester Polytechnic Institute)

This repository provides a pipeline for benchmarking vision based grasp detection algorithms. The pipeline performs pick and place of objects based on 3DOF/6DOF grasps detected by the grasp detection algorithms. The repository was tested with Franka Panda robot with eye in hand realsense camera. The pointcloud/depth image of the workspase is captured from the birdseye view. It is also assumed that the gripper approaches the object only from the top. These constraints will be lifted eventually. 

Refer the [simulator repository](https://github.com/cdbharath/franka_panda "simulator repository") to run the benchmarking procedures in the simulation environment. 

Refer the [grasp detection algorithms](https://github.com/cdbharath/grasp_synthesis "grasp detection algorithms") to find the algorithms that we tested with this pipeline

## How to run (Perferably in order):
```
# Run any one of the commands as per the requirement
roslaunch benchmarking_grasp run_benchmark.launch  # To run on real robot, depth input                                             
roslaunch benchmarking_grasp run_benchmark.launch sim_mode:=true  # To run in simulator, depth input
roslaunch benchmarking_grasp run_benchmark.launch point_cloud_input:=true align_depth:=false  # For point cloud input
roslaunch benchmarking_grasp run_benchmark.launch point_cloud_input:=true align_depth:=false  # To run in simulator, point cloud input

### In Franka Panda Robot (Check simulator repository to use simulator instead)
roscore
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<ip> load_gripper:=true
rosrun moveit_adapter moveit_adapter.py

### Check grasp detection algorithms repository to use the algorithms
rosrun <grasp_det_algo> service_server.py  
```

## Features 
1. Script to find depth ROI (```roslaunch benchmarking_grasp find_depth_roi.launch <args>:=<values>```)
2. Script to find point cloud ROI (```roslaunch benchmarking_grasp find_point_cloud_roi.launch <args>:=<values>```)
3. ```configuration.yaml``` exposes pipeline parameters that can be tuned as per our requirements. ```example_configuration``` has sample configuration files that were used for the experimemnts.
4. ```benchmarking.yaml``` exposes parameters specific to the benchmarking protocols and objects to be simulated
5. The configuration file and launch file arguments can be used to turn on/off the features such as depth completion, filtering ROI, recording video e.t.c.
