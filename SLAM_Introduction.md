# SLAM部分
* ubuntu 16.04 LTS
* Intel® Core™ i7-5500U CPU @ 2.40GHz × 4 
* Intel Realsense SDK2.0
## Package:img_publisher:
img_publisher package主要功能是通过D435驱动接收相机数据，将深度图和RGB图对齐后，使用cv_bridge将图像信息转换为ROS消息，然后建立节点并发送话题。同时，生成当前相机坐标系下的点云数据并发送话题。