# tkdnn_ros
## Overview
This is a ROS package developed for object detection in camera images. You can use the acceleration library tkDNN on ROS.The documentation and code are not well maintained now, but we will continue to do so in the future.

## Use your own detection objects
In order to use your own detection objects you need to provide your weights file(trt file like yolov4.trt) inside the directories
```
catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/weights/
```

## Run
```
catkin build
```
```
roslaunch tkdnn_ros tkdnn_ros.launch
```
