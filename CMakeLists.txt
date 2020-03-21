cmake_minimum_required(VERSION 2.8.3)
project(yolo_tensorrt_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  sensor_msgs
  cv_bridge
  image_transport
)

## Look for gflags
find_package(gflags REQUIRED)

if(gflags_FOUND)
message("-- gflags found")
endif()

## Look for CUDA
find_package(CUDA REQUIRED)
message("-- CUDA version:$(CUDA_VERSION)")

if(CUDA_VERSION_MAJOR GREATER 9)
	message("-- CUDA ${CUDA_VERSION_MAJOR} detected")
	set(
		CUDA_NVCC_FLAGS
		${CUDA_NVCC_FLAGS}; 
		-gencode arch=compute_53,code=sm_53
	)
find_package(OpenCV REQUIRED)
endif()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yolo_tensorrt_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE sources 
  src/yolo-tensorrt/modules/*.hpp
  src/yolo-tensorrt/modules/*.cpp
  src/yolo-tensorrt/modules/*.h
  src/yolo-tensorrt/modules/*.cu
  src/yolo-tensorrt/extra/*.h
)

add_executable(talker src/talker.cpp)


cuda_add_library(detector SHARED ${sources})
target_include_directories(detector PRIVATE src/yolo-tensorrt/modules/ src/yolo-tensorrt/extra/ D:/dependency/gflag/include ${CUDA_TOOLKIT_ROOT_DIR}/include D:/dependency/TensorRT-5.1.5.0_cuda-10.0_cudnn-7.5/include D:/dependency/OPENCV341/include ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

target_include_directories(talker PRIVATE src/yolo-tensorrt/modules/ src/yolo-tensorrt/extra/)
target_link_libraries(detector nvinfer nvinfer_plugin nvcaffe_parser gflags "stdc++fs")


if(CUDA_VERSION_MAJOR GREATER 9)
	target_link_libraries(talker detector opencv_core opencv_calib3d opencv_dnn ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif()

add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)


## define executable for image converter
add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(image_converter ${PROJECT_NAME}_generate_messages_cpp)

## prepare object detector library target
add_library(objDetector src/obj_detector/obj_detector.cpp)
target_include_directories(objDetector PRIVATE src/yolo-tensorrt/modules/ src/yolo-tensorrt/extra/ D:/dependency/gflag/include ${CUDA_TOOLKIT_ROOT_DIR}/include D:/dependency/TensorRT-5.1.5.0_cuda-10.0_cudnn-7.5/include D:/dependency/OPENCV341/include ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
target_link_libraries(objDetector nvinfer nvinfer_plugin nvcaffe_parser gflags "stdc++fs")

## define executable for vision node
add_executable(vision src/visionNode.cpp)
target_include_directories(vision PRIVATE src/yolo-tensorrt/modules/ src/yolo-tensorrt/extra/)
target_link_libraries(vision ${catkin_LIBRARIES} objDetector)