# tkdnn_ros
This is a repository that makes [tkDNN](https://github.com/ceccocats/tkDNN) run on ROS.

## Dependencies
This branch works on every NVIDIA GPU that supports the dependencies:
- CUDA 10.0
- CUDNN 7.603
- TENSORRT 6.01
- OPENCV 3.4
- yaml-cpp 0.5.2 (sudo apt install libyaml-cpp-dev)

## Preparation
1. Replace the tkdnn repository with the tkdnn folder in this repository.
2. Make environment. Please follow Instructions of tkDNN repository.
3. Make TensorRT inference engine(trt file).


## Use your own detection objects
1. Place your custom trt file in a hierarchy like the one below.
```
catkin_ws/src/tkdnn_ros/tkdnn_ros/yolo_network_config/weights/
```
2. Edit config file.
```
catkin_ws/src/tkdnn_ros/tkdnn_ros/config/weights/tkdnn.yaml
```
- Write a video path that you want to infer.
- Write trt file path.
- Write a threshold.
- Write the number of classes


## Run
```
catkin build
```
```
roslaunch tkdnn_ros tkdnn_ros.launch
```
