# SLAM部分
* ubuntu 16.04 LTS
* Intel® Core™ i7-5500U CPU @ 2.40GHz × 4 
* Intel Realsense SDK2.0
## 1. Package:img_publisher
img_publisher包主要功能是通过D435驱动接收相机数据，将深度图和RGB图对齐后，使用cv_bridge将图像信息转换为ROS消息，然后建立节点并发送话题。同时，生成当前相机坐标系下的点云数据并发送话题。其中代码主体部分位于[publisher.cpp](src/img_publisher/src/publisher.cpp)文件中。以下是具体的代码实现：
### 1.1. 声明及初始化
main函数开始之前，声明函数以及常量；定义点云类型。
```
// 相机图像接收频率
#define FPS 30

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 获取深度像素对应长度单位转换
float get_depth_scale(rs2::device dev);

// 检查摄像头数据管道设置是否改变
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
```
### 1.2. 相机管道配置
在realsense SDK2.0中，
```
    // 创建一个管道以及管道的参数变量
    rs2::pipeline pipe;
    rs2::config p_config;

    //配置管道以及启动相机
    p_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, FPS);
    p_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, FPS);
    p_config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, FPS);
    rs2::pipeline_profile profile = pipe.start(p_config);
```
