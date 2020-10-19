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

## Use your own detection objects
In order to use your own detection objects you need to provide your weights file(trt file like yolov4.trt) inside the directories
```
catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/
```

## Run
```
catkin build
```
```
roslaunch tkdnn_ros tkdnn_ros.launch
```
