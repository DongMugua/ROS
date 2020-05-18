# SLAM部分
* ubuntu 16.04 LTS
* Intel® Core™ i7-5500U CPU @ 2.40GHz × 4 
* Intel Realsense SDK2.0
[toc]
## 1. Package: img_publisher
img_publisher包主要功能是通过D435驱动接收相机数据，将深度图和RGB图对齐后，使用cv_bridge将图像信息转换为ROS消息，然后建立节点并发送话题。同时，生成当前相机坐标系下的点云数据并发送话题。其中代码主体部分位于[publisher.cpp](src/img_publisher/src/publisher.cpp)文件中。以下是具体的代码实现：
### 1.1 初始化和配置
在主要功能实现之前，我们进行了函数、变量的初始化，相机的配置以及和ROS相关的配置。
#### 1.1.1 函数、参数的声明及初始化
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
#### 1.1.2 相机管道配置以及深度图像向RGB图像的对齐
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
#### 1.1.3 相机内参、外参的获取
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
#### 1.1.4 ROS相关的配置
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
由于需要实时持续获取相机数据，我们将主要功能代码写在的while循环中。只有当节点关闭时跳出循环。
#### 1.2.1 帧的获取以及深度帧的对齐
当有图像帧被接受时，wait_for_frames()函数返回图像帧到**frameset**变量中，之后我们分别获取RGB图像帧和深度图像帧。在判断过摄像头数据管道设置没有改变之后，利用**align**将深度图像对齐到RGB图像上面并且得到对齐之后的**processed**变量。
*注意，如果此时加上apply_filter(rs2::colorizer c)色彩滤波器，在RVIZ中可以直接看到彩色的深度图像，但是如果将此图像作为ROS消息传送到RGB-D后，无法得到理想的点云图。*
```
    rs2::frameset frameset = pipe.wait_for_frames();

    const rs2::frame &color_frame = frameset.get_color_frame();
    const rs2::frame &depth_frame = frameset.get_depth_frame();

    auto color_format_ = color_frame.as<rs2::video_frame>().get_profile().format();
    auto swap_rgb_ = color_format_ == RS2_FORMAT_BGR8 || color_format_ == RS2_FORMAT_BGRA8;
    auto nb_color_pixel_ = (color_format_ == RS2_FORMAT_RGB8 || color_format_ == RS2_FORMAT_BGR8) ? 3 : 4;

    // 因为rs2::align 正在对齐深度图像到其他图像流，我们要确保对齐的图像流不发生改变
    if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
    {
        std::cout<<"changed?"<<std::endl;
        //如果profile发生改变，则更新align对象，重新获取深度图像像素到长度单位的转换比例
        profile = pipe.get_active_profile();
        align = rs2::align(align_to);
        depth_scale = get_depth_scale(profile.get_device());
    }

    // 获取对齐后的帧
    rs2::frameset processed = align.process(frameset);

    // 尝试获取对齐后的深度图像帧和其他帧
    rs2::frame aligned_color_frame = processed.get_color_frame();
    rs2::frame aligned_depth_frame = processed.get_depth_frame(); // apply_filter(c)

    // 获取图像的宽高
    const int depth_w = aligned_depth_frame.as<rs2::video_frame>().get_width();
    const int depth_h = aligned_depth_frame.as<rs2::video_frame>().get_height();
    const int color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
    const int color_h = aligned_color_frame.as<rs2::video_frame>().get_height();
```
#### 1.2.2 RGB、深度图像消息发布
发布时，RGB图像为8位3通道(CV_8UC3)，名为**rgb8**,深度图像为16位单通道(CV_16UC1),名为**16UC1**。
```
    // 获取时间戳
    imgHeader.stamp = ros::Time::now();

    // RGB图像
    cv::Mat aligned_color_image(cv::Size(color_w,color_h), CV_8UC3, (void*)aligned_color_frame.get_data(), cv::Mat::AUTO_STEP);
    rgbMsg = cv_bridge::CvImage(imgHeader, "rgb8", aligned_color_image).toImageMsg();
    rgbPub.publish(rgbMsg);
    
    // 深度图像
    cv::Mat aligned_depth_image(cv::Size(depth_w,depth_h), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
    depthMsg = cv_bridge::CvImage(imgHeader, "16UC1", aligned_depth_image).toImageMsg();
    depthPub.publish(depthMsg);

```
RGB图和深度图示例：
![image](https://github.com/DongMugua/TRY/blob/master/Imgae/RGB%20%26%20Depth.png)
#### 1.2.3 点云图像发布
点云图像是基于函数rs2_deproject_pixel_to_point()生成的。其中运用了针孔相机成像的转换矩阵。在位置坐标赋值之后，又将相对应的颜色坐标赋值到对应的点上，从而能够得到彩色的点云图。
```
    // 点云
    rs2::video_frame vf = aligned_depth_frame.as<rs2::video_frame>();

    const int width = vf.get_width();
    const int height = vf.get_height();
    pointcloud_->width = (uint32_t)width;
    pointcloud_->height = (uint32_t)height;
    pointcloud_->resize((size_t)(width * height));

    const uint16_t *p_depth_frame = reinterpret_cast<const uint16_t *>(aligned_depth_frame.get_data());
    const unsigned char *p_color_frame = reinterpret_cast<const unsigned char *>(aligned_color_frame.get_data());

    for (int i = 0; i < height; i++)
    {
        auto depth_pixel_index = i * width;
        for (int j = 0; j < width; j++, depth_pixel_index++)
        {
            if (p_depth_frame[depth_pixel_index] == 0)
            {
                pointcloud_->points[(size_t)depth_pixel_index].x = m_invalid_depth_value_;
                pointcloud_->points[(size_t)depth_pixel_index].y = m_invalid_depth_value_;
                pointcloud_->points[(size_t)depth_pixel_index].z = m_invalid_depth_value_;
            }

            // Get the depth value of the current pixel
            auto pixels_distance =  depth_scale *p_depth_frame[depth_pixel_index];
            float depth_point[3];
            const float pixel[] = {(float)j, (float)i};

            rs2_deproject_pixel_to_point(depth_point, &m_depth_intrinsics_, pixel, pixels_distance);
            
            if (pixels_distance > m_max_z_)
                depth_point[0] = depth_point[1] = depth_point[2] = m_invalid_depth_value_;

            pointcloud_->points[(size_t)depth_pixel_index].x = depth_point[2];
            pointcloud_->points[(size_t)depth_pixel_index].y = -depth_point[0];
            pointcloud_->points[(size_t)depth_pixel_index].z = -depth_point[1];

            float color_point[3];
            rs2_transform_point_to_point(color_point, &m_depth_2_color_extrinsics_, depth_point);
            float color_pixel[2];
            rs2_project_point_to_pixel(color_pixel, &m_color_intrinsics_, color_point);

            if (color_pixel[1] < 0 || color_pixel[1] >= color_height_ || color_pixel[0] < 0 || color_pixel[0] >= color_width_)
            {
                pointcloud_->points[(size_t)depth_pixel_index].x = m_invalid_depth_value_;
                pointcloud_->points[(size_t)depth_pixel_index].y = m_invalid_depth_value_;
                pointcloud_->points[(size_t)depth_pixel_index].z = m_invalid_depth_value_;
            }
            else
            {
                unsigned int i_ = (unsigned int)color_pixel[1];
                unsigned int j_ = (unsigned int)color_pixel[0];
                if (swap_rgb_)
                {
                    pointcloud_->points[(size_t)depth_pixel_index].b =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_];
                    pointcloud_->points[(size_t)depth_pixel_index].g =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_ + 1];
                    pointcloud_->points[(size_t)depth_pixel_index].r =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_ + 2];
                }
                else
                {
                    pointcloud_->points[(size_t)depth_pixel_index].r =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_];
                    pointcloud_->points[(size_t)depth_pixel_index].g =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_ + 1];
                    pointcloud_->points[(size_t)depth_pixel_index].b =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_ + 2];
                }

            }
        }
    }
    pcl::toROSMsg(*pointcloud_, msg_pointcloud);
    msg_pointcloud.header.stamp = imgHeader.stamp;
    msg_pointcloud.is_dense = true;
    msg_pointcloud.header.frame_id = "map";

    pointcloud_publisher_.publish(msg_pointcloud);
    pointcloud_ ->clear();
    ros::spinOnce();
```
点云图示例：
![image](https://github.com/DongMugua/TRY/blob/master/Imgae/PointCloud.png)
### 1.3 结果分析
#### 1.3.1 ROS节点图
![image](https://github.com/DongMugua/TRY/blob/master/Imgae/rqt_graph_publisher.png）
到此为止，我们便创建了publisher节点并发布了
## 附录
* [ORB-SLAM2稠密点云重建:RGBD室内](https://blog.csdn.net/qq_41524721/article/details/79126062)
* [realsense_camera.cpp](https://github.com/abhishek098/realsense_camera_wrapper/blob/master/src/realsense_camera_wrapper/realsense_camera.cpp)
* [Sample Code for Intel® RealSense™ cameras](https://dev.intelrealsense.com/docs/code-samples)
* [realsense SDK2.0学习：：（一）读取D435视频【彩色&&深度】](https://blog.csdn.net/dieju8330/article/details/85272800)
* [realsense SDK2.0学习：：（二）D435深度图片对齐到彩色图片-SDK实现](https://blog.csdn.net/dieju8330/article/details/85272919?utm_medium=distribute.pc_relevant.none-task-blog-baidujs-2)
## Q
1. realsense1和realsense2区别
2. realsense2直接获取相机内参？
