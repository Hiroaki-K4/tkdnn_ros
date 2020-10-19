# tkdnn_ros
This is a repository that makes [tkDNN](https://github.com/ceccocats/tkDNN) run on ROS.


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
