#ifndef OBSERVER_H
#define OBSERVER_H
#include <iostream>
#include <cstdlib>

// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <KeyFrame.h>
#include <System.h>
#include <Tracking.h>
#include <Converter.h>
#include <vector>
#include <thread>

#include <opencv2/core/core.hpp>
// #include <tf_conversions/tf_eigen.h>

using namespace std;
using namespace ORB_SLAM2;
// using namespace cv;

class MyPclMapping
{
/*
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    // 类对象的构造函数
   // MyPclMapping();

    
    // 获取新的关键帧函数
    void insertKF(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);
    // 获取关键帧对应的点云地图，并将新旧地图叠加
    void generatepcl(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);

    PointCloud::Ptr Getglobalmap()
    {
        unique_lock<mutex> locker(generation);
        return globalMap;
    }

    void setcurrentCamera(cv::Mat Rwc, cv::Mat twc);

private:
    mutex generation;
    vector<cv::Mat> rgbs;
    vector<cv::Mat> depthes;
    vector<KeyFrame *> kfs;

    PointCloud::Ptr globalMap;

    Eigen::Vector3d kfplace;
    // 使用PCLVisualizer对象来显示地图
    // pcl::visualization::PCLVisualizer showcamera;
    Eigen::Vector3d currentcamera;
    cv::Mat currentTwc;
    */
};

#endif