#!/bin/bash
set -e

# setup ros environment
source /opt/ros/melodic/setup.bash

echo "source /opt/ros/melodic/setup.zsh" >> /root/.zshrc
echo "source /root/catkin_ws/devel/setup.zsh" >> /root/.zshrc
echo "export ROS_PACKAGE_PATH=/opt/ros/melodic/share:/root/catkin_ws/src/DS-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM" >> /root/.zshrc

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/catkin_ws/src/DS-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM

# echo "build ROS pacagkes"
# cd /root/catkin_ws 
# chmod +x /root/catkin_ws/src/octomap_mapping/octomap_server/cfg/OctomapServer.cfg

# build caffe
# cd /root/caffe-segnet \
#     && [ ! -d "build" ] && mkdir build  \
#     && cd build \
#     && cmake .. -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX=/usr/local -DUSE_CUDNN=OFF \
#     && make -j2 \
#     && sudo make install

# echo "Build DSSLAM"
cd /root/catkin_ws/src/DS-SLAM
source ~/.zshrc  && ./DS_SLAM_BUILD.sh
source /root/catkin_ws/devel/setup.bash

# cd /root/catkin_ws &&  source /opt/ros/melodic/setup.bash && catkin_make

exec "$@"

