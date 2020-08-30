# Demo

<https://www.bilibili.com/video/av59793400?from=search&seid=12598277376962486068>

# Change Log

- support CUDA 10
- Find caffe (caffe-segnet) package in CMakeList
- Add docker image

# Build

## Environment
- Ubuntu 16.04 (Some problems in Ubuntu 18)
- ROS Kinetic
- Cuda 10.0
- Cudnn 7
- Caffe (SegNet)
- Eigen 3
- OpenCV 3
- [optional] docker: 19.03.12 

## Build and install *caffe-segnet-cudnn5*

```sh
git clone https://github.com/yubaoliu/caffe-segnet.git 
cd SegNet/caffe-segnet
mkdir build
cd build
cmake .. -CMAKE_INSTALL_PREFIX=/usr/local
make & make install
```

## Build DS-SLAM

```sh
git clone https://github.com/yubaoliu/DS-SLAM.git
```

# ROS~PACKAGEPATH

```sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/catkin_ws/src/DS-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM
```

# Caffe Modle

segnet_pascal.caffemodel

```sh
http://mi.eng.cam.ac.uk/~agk34/resources/SegNet/segnet_pascal.caffemodel

or

gdlink 'https://drive.google.com/uc\?export\=download\&id\=1KzyEpaLovCyDKEj-LkAGTT6l-s3IRpSG' | xargs -n1 wget -c -O ./segnet_pascal.caffemodel

https://drive.google.com/file/d/1KzyEpaLovCyDKEj-LkAGTT6l-s3IRpSG/view?usp=sharing
```

Please download and place the folder in the same path with the folder of
prototxts and tools.

# Run

Customize dataset and associate path in launch file

```sh
roslaunch fr3_walk_half.launch
```

# Docker

```sh
docker-compose build
docker-compose up

```

