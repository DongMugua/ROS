#include <cv.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>

// 相机图像接收频率
#define FPS 30

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 获取深度像素对应长度单位转换
float get_depth_scale(rs2::device dev);

// 检查摄像头数据管道设置是否改变
bool profile_changed(const std::vector<rs2::stream_profile> &current, const std::vector<rs2::stream_profile> &prev);

float m_invalid_depth_value_ = 0.0;
float m_max_z_ = 8.0;

// pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

int main(int argc, char **argv)
{
    // 声明滤波器，
    // rs2::colorizer c;
    // Create a pipeline to easily configure and start the camera
    // 创建一个管道以及管道的参数变量
    rs2::pipeline pipe;
    rs2::config p_config;

    // 配置管道以及启动相机
    p_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, FPS);
    p_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, FPS);
    // p_config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, FPS);
    rs2::pipeline_profile profile = pipe.start(p_config);

    // 使用数据管道的profile获取深度图像像素对应于长度单位（米）的转换比例
    float depth_scale = get_depth_scale(profile.get_device());
    std::cout << "depth_scale: " << depth_scale << std::endl;
    // "align_to"是我们打算用深度图像对齐的图像流
    // 选择彩色图像数据流来作为对齐对象
    rs2_stream align_to = RS2_STREAM_COLOR;
    //rs2::align 允许我们去实现深度图像对齐其他图像
    rs2::align align(align_to);

    // Define a variable for controlling the distance to clip
    // float depth_clipping_distance = 1.f;

    // 声明数据流
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    // 获取深度相机内参
    rs2_intrinsics m_depth_intrinsics_ = depth_stream.get_intrinsics();
    // 获取rgb相机内参
    rs2_intrinsics m_color_intrinsics_ = color_stream.get_intrinsics();
    // 获取深度相机相对rgb相机的外参，即变换矩阵
    rs2_extrinsics m_depth_2_color_extrinsics_ = depth_stream.get_extrinsics_to(color_stream);
    // 获取rgb帧的长宽
    auto color_width_ = m_color_intrinsics_.width;
    auto color_height_ = m_color_intrinsics_.height;

    std::cout << "depth.fx  " << m_depth_intrinsics_.fx << std::endl;
    std::cout << "depth.fy  " << m_depth_intrinsics_.fy << std::endl;
    std::cout << "depth.ppx " << m_depth_intrinsics_.ppx << std::endl;
    std::cout << "depth.ppy " << m_depth_intrinsics_.ppy << std::endl;

    std::cout << "color.fx  " << m_color_intrinsics_.fx << std::endl;
    std::cout << "color.fy  " << m_color_intrinsics_.fy << std::endl;
    std::cout << "color.ppx " << m_color_intrinsics_.ppx << std::endl;
    std::cout << "color.ppy " << m_color_intrinsics_.ppy << std::endl;

    std::cout << "ex.fx  " << m_depth_2_color_extrinsics_.rotation[0] << std::endl;
    std::cout << "ex.fx  " << m_depth_2_color_extrinsics_.rotation[1] << std::endl;
    std::cout << "ex.fy  " << m_depth_2_color_extrinsics_.translation[0] << std::endl;
    std::cout << "ex.fy  " << m_depth_2_color_extrinsics_.translation[1] << std::endl;
    std::cout << "ex.fy  " << m_depth_2_color_extrinsics_.translation[2] << std::endl;

    ros::init(argc, argv, "image_publisher");

    std::cout << "node init " << std::endl;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher rgbPub = it.advertise("camera/rgb/image_raw", 1);
    image_transport::Publisher depthPub = it.advertise("camera/depth_registered/image_raw", 1);
    ros::Publisher pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_in", 1);

    sensor_msgs::ImagePtr rgbMsg, depthMsg;
    std_msgs::Header imgHeader = std_msgs::Header();

    // PointCloud::Ptr cloud(new PointCloud);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_;
    PointCloud::Ptr pointcloud_ = boost::make_shared<PointCloud>();
    sensor_msgs::PointCloud2 msg_pointcloud;

    // pcl::VoxelGrid<PointT> voxel;
    // voxel.setLeafSize(0.01f,0.01f,0.01f);
    // // voxel.setLeafSize(0.1f,0.1f,0.1f);

    std::cout << "RUNNING..." << std::endl;

    while (nh.ok())
    {

        rs2::frameset frameset = pipe.wait_for_frames();

        const rs2::frame &color_frame = frameset.get_color_frame();
        const rs2::frame &depth_frame = frameset.get_depth_frame();

        auto color_format_ = color_frame.as<rs2::video_frame>().get_profile().format();
        auto swap_rgb_ = color_format_ == RS2_FORMAT_BGR8 || color_format_ == RS2_FORMAT_BGRA8;
        auto nb_color_pixel_ = (color_format_ == RS2_FORMAT_RGB8 || color_format_ == RS2_FORMAT_BGR8) ? 3 : 4;

        // 因为rs2::align 正在对齐深度图像到其他图像流，我们要确保对齐的图像流不发生改变
        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            std::cout << "changed?" << std::endl;
            //如果profile发生改变，则更新align对象，重新获取深度图像像素到长度单位的转换比例
            profile = pipe.get_active_profile();
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }

        // 获取对齐后的帧
        rs2::frameset processed = align.process(frameset);

        // 尝试获取对齐后的深度图像帧和其他帧
        rs2::frame aligned_color_frame = processed.get_color_frame(); //processed.first(align_to);
        rs2::frame aligned_depth_frame = processed.get_depth_frame(); //.apply_filter(c);;//.apply_filter(c);;

        // 获取图像的宽高
        const int depth_w = aligned_depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h = aligned_depth_frame.as<rs2::video_frame>().get_height();
        const int color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
        const int color_h = aligned_color_frame.as<rs2::video_frame>().get_height();

        // 获取时间戳
        imgHeader.stamp = ros::Time::now();

        // RGB图像
        cv::Mat aligned_color_image(cv::Size(color_w, color_h), CV_8UC3, (void *)aligned_color_frame.get_data(), cv::Mat::AUTO_STEP);
        rgbMsg = cv_bridge::CvImage(imgHeader, "rgb8", aligned_color_image).toImageMsg();
        rgbPub.publish(rgbMsg);

        // 深度图像
        cv::Mat aligned_depth_image(cv::Size(depth_w, depth_h), CV_16UC1, (void *)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
        depthMsg = cv_bridge::CvImage(imgHeader, "16UC1", aligned_depth_image).toImageMsg();
        depthPub.publish(depthMsg);

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
                auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];
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
        // PointCloud filtered_cloud;

        // //sor.setInputCloud(cloud);
        // //sor.filter(filtered_cloud);

        // //cloud->swap(filtered_cloud);

        // voxel.setInputCloud(cloud);
        // voxel.filter(filtered_cloud);

        // sensor_msgs::PointCloud2 msg_pointcloud;
        // pcl::toROSMsg(filtered_cloud,msg_pointcloud);
        // //pcl::toROSMsg(*cloud,msg_pointcloud);

        // msg_pointcloud.header.stamp = imgHeader.stamp;
        // msg_pointcloud.is_dense = true;
        // msg_pointcloud.header.frame_id = "base_link";

        // //sor.filter(filtered_pointcloud);
        // pointcloud_publisher_.publish(msg_pointcloud);

        pcl::toROSMsg(*pointcloud_, msg_pointcloud);
        msg_pointcloud.header.stamp = imgHeader.stamp;
        msg_pointcloud.is_dense = true;
        msg_pointcloud.header.frame_id = "map";

        pointcloud_publisher_.publish(msg_pointcloud);
        pointcloud_->clear();
        ros::spinOnce();
    }
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor &sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
}

bool profile_changed(const std::vector<rs2::stream_profile> &current, const std::vector<rs2::stream_profile> &prev)
{
    for (auto &&sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile &current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}
