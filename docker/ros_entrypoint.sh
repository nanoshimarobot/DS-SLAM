#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
# source "/home/ubuntu/catkin_ws/devel/setup.bash"

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/catkin_ws/src/DS-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM


# echo "compile caffe-segnet"
# cd /root/caffe-segnet
# [ ! -d "build" ] && mkdir build 
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX=/usr/local
# make -j2 && make install
 
# echo "build ROS pacagkes"
# cd /root/catkin_ws 
# chmod +x /root/catkin_ws/src/octomap_mapping/octomap_server/cfg/OctomapServer.cfg
# catkin_make

# echo "Build DSSLAM"
cd /root/catkin_ws/src/DS-SLAM
bash DS_SLAM_BUILD.sh

exec "$@"

