Make_Threads=1
OpenCV_Version=3.1.0
sed -i -e 's/-j/-j'$Make_Threads'/' -e 's/\$(nproc)//' ./build.sh
sed -i -e 's/-march=native//' -e 's/OpenCV 2.4.3/OpenCV '$OpenCV_Version'/g' ./CMakeLists.txt
sed -i -e 's/-march=native//' -e 's/OpenCV 2.4.3/OpenCV '$OpenCV_Version'/g' ./Examples/ROS/ORB_SLAM2/CMakeLists.txt
sed -i 's/-march=native//' ./Thirdparty/DBoW2/CMakeLists.txt
sed -i 's/-march=native//' ./Thirdparty/g2o/CMakeLists.txt
echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$PWD/Examples/ROS
mkdir -p Examples/ROS/ORB_SLAM2/build && cd Examples/ROS/ORB_SLAM2/build
cmake .. && make" >> ./build.sh
