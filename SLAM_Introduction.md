# SLAM部分
* ubuntu 16.04 LTS
* Intel® Core™ i7-5500U CPU @ 2.40GHz × 4 
* Intel Realsense SDK2.0
## Package:img_publisher
img_publisher包主要功能是通过D435驱动接收相机数据，将深度图和RGB图对齐后，使用cv_bridge将图像信息转换为ROS消息，然后建立节点并发送话题。同时，生成当前相机坐标系下的点云数据并发送话题。其中代码主体部分位于[publisher.cpp](src/img_publisher/src/publisher.cpp)文件中。以下是具体的代码实现：
### 声明及初始化
```
#define FPS 30

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//获取深度像素对应长度单位转换
float get_depth_scale(rs2::device dev);

//检查摄像头数据管道设置是否改变
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

float m_invalid_depth_value_ = 0.0;
float m_max_z_ = 8.0;
```
