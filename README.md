# Benchmarking Vision based Grasp Detection algorithms

Research under Prof Berk Calli (Manipulation and Environmental Robotics Lab, Worcester Polytechnic Institute)

This repository is built on top of [Franka Panda](https://github.com/cdbharath/franka_panda "Franka Panda") manipulation pipeline repository. 
Refer [Vision based Grasp Detection Algorithms](https://github.com/cdbharath/learning_based_grasp_synthesis "Vision based Grasp Detection Algorithms") for sample algorithms to test in this benchmark environment 

## How to run:
```
### In the Remote PC
roscore
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=172.16.0.2 load_gripper:=true
roslaunch pick_and_place publish_transform.launch
rosrun benchmarking_grasp test_grasp_transform.py
rosrun benchmarking_grasp test_benchmark_pipeline.py

### In the operating PC
roslaunch realsense2_camera rs_camera.launch align_depth:=true
rosrun ggcnn service_server.py
```

## References:
1. [Franka Panda](https://github.com/cdbharath/franka_panda "Franka Panda")
2. [Vision based Grasp Detection Algorithms](https://github.com/cdbharath/learning_based_grasp_synthesis "Vision based Grasp Detection Algorithms")
