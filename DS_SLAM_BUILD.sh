echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
[ ! -d "buid" ] && mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j2

cd ../../g2o

echo "------------------------------------------------"
echo "Configuring and building Thirdparty/g2o ..."

[ ! -d "buid" ] && mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j2

cd ../../../

echo "------------------------------------------------"
echo "Configuring and building libsegmentation.so ..."

cd Examples/ROS/ORB_SLAM2_PointMap_SegNetM/libsegmentation
[ ! -d "buid" ] && mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j2

cd ../../../../../

echo "------------------------------------------------"
echo "Configuring and building libORB_SLAM2_PointMap_SegNetM.so ..."

[ ! -d "buid" ] &&  mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j2

cd ..


echo "------------------------------------------------"
echo "ROS: Configuring and building Executable TUM ..."

cd Examples/ROS/ORB_SLAM2_PointMap_SegNetM
[ ! -d "buid" ] && mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j2

cd ../../../../

