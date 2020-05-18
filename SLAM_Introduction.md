# SLAM部分
* ubuntu 16.04 LTS
* Intel® Core™ i7-5500U CPU @ 2.40GHz × 4 
* Intel Realsense SDK2.0
## 1. Package: img_publisher
img_publisher包主要功能是通过D435驱动接收相机数据，将深度图和RGB图对齐后，使用cv_bridge将图像信息转换为ROS消息，然后建立节点并发送话题。同时，生成当前相机坐标系下的点云数据并发送话题。其中代码主体部分位于[publisher.cpp](src/img_publisher/src/publisher.cpp)文件中。以下是具体的代码实现：
### 1.1 函数、参数的声明及初始化
在进入main函数之前，声明函数、初始化变量；定义常量；定义点云类型。
```
    // 相机图像接收频率
    #define FPS 30

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 获取深度像素对应长度单位转换
    float get_depth_scale(rs2::device dev);

    // 检查摄像头数据管道设置是否改变
    bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

    float m_invalid_depth_value_ = 0.0;
    float m_max_z_ = 8.0;
```
#### 1.1.1 相机管道配置以及深度图像向RGB图像的对齐
在realsense SDK2.0中，是通过管道获取相机的RGB帧和深度帧。所以初始化时，我们配置了两个数据流——16位单通道的深度数据流和8位三通道的RGB数据流，以及30Hz的接收频率。
```
    // 创建一个管道以及管道的参数变量
    rs2::pipeline pipe;
    rs2::config p_config;

    // 配置管道以及启动相机
    p_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, FPS);
    p_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, FPS);
    rs2::pipeline_profile profile = pipe.start(p_config);
```
因为每个深度相机的深度像素单位可能不同，因此我们在这里获取它：
```
    // 使用数据管道的profile获取深度图像像素对应于长度单位（米）的转换比例
    float depth_scale = get_depth_scale(profile.get_device());
```
在此，我们需要申明一个能够实现深度图向其他图像对齐的rs2::align类型的变量**align**，在后续的代码中，我们将通过此变量实现深度帧的对齐。
```
    // "align_to"是我们打算用深度图像对齐的图像流
    // 选择RGB图像数据流来作为对齐对象
    rs2_stream align_to = RS2_STREAM_COLOR; 
    //rs2::align 允许我们去实现深度图像对齐其他图像
    rs2::align align(align_to);
```
#### 1.1.2 相机内参、外参的获取
在realsense2中，有直接获取相机内参外参的接口：

```
    // 声明数据流
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    // 获取深度相机内参
    rs2_intrinsics m_depth_intrinsics_ = depth_stream.get_intrinsics();
    // 获取RGB相机内参
    rs2_intrinsics m_color_intrinsics_ = color_stream.get_intrinsics();
    // 获取深度相机相对于RGB相机的外参，即变换矩阵
    rs2_extrinsics  m_depth_2_color_extrinsics_ = depth_stream.get_extrinsics_to(color_stream);
    // 获取rgb帧的长宽
    auto color_width_ = m_color_intrinsics_.width;
    auto color_height_ = m_color_intrinsics_.height;
```
#### 1.1.3 ROS相关的配置
```
    ros::init(argc, argv, "image_publisher");

    // ROS节点声明
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher rgbPub = it.advertise("camera/rgb/image_raw", 1);
    image_transport::Publisher depthPub = it.advertise("camera/depth_registered/image_raw", 1);
    ros::Publisher pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_in", 1);
    
    // 图像消息声明
    sensor_msgs::ImagePtr rgbMsg, depthMsg;
    std_msgs::Header imgHeader = std_msgs::Header();
    // 点云消息声明
    PointCloud::Ptr pointcloud_ = boost::make_shared< PointCloud >( );
    sensor_msgs::PointCloud2 msg_pointcloud;
```
### 1.2 主要功能实现

## 附录
* [ORB-SLAM2稠密点云重建:RGBD室内](https://blog.csdn.net/qq_41524721/article/details/79126062)
* [Sample Code for Intel® RealSense™ cameras](https://dev.intelrealsense.com/docs/code-samples)
* [rs-hello-realsense.cpp](https://github.com/IntelRealSense/librealsense/blob/master/examples/hello-realsense/rs-hello-realsense.cpp)
* [realsense SDK2.0学习：：（一）读取D435视频【彩色&&深度】](https://blog.csdn.net/dieju8330/article/details/85272800)
* [realsense SDK2.0学习：：（二）D435深度图片对齐到彩色图片-SDK实现](https://blog.csdn.net/dieju8330/article/details/85272919?utm_medium=distribute.pc_relevant.none-task-blog-baidujs-2)

## Q
1. realsense1和realsense2区别
2. realsense2直接获取相机内参？
