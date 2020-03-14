# YOLO TensorRT ROS

## Introduction
This is a ROS package developed for object detection in camera images. This is based on [enazoe's yolo-tensorrt](https://github.com/enazoe/yolo-tensorrt) wrapper. Testing and development is currently taking place on the Nvidia Jetson Nano.

## Prerequisites

For running this on a Nvidia Jetson Nano:

* Nvidia JetPack 4.2.2 (system image)
* gflags

## Downloading Pretrained Models

Download and place the darknet .cfg and .weights files into the `src/config` directory. Then update the program accordingly to use the desired network config and weights.

