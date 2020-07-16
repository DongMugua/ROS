/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <condition_variable>

using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
     
    PointCloudMapping( double resolution_ );
    
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();
    void viewer();

    // 观察器：
    // 获取新的关键帧函数
    void insertKF(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);
    // 获取关键帧对应的点云地图，并将新旧地图叠加
    void generatepcl(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);

    PointCloud::Ptr Getglobalmap()
    {
        unique_lock<mutex> locker(generation);
        return globalMapS;
    }

    void setcurrentCamera(cv::Mat Rwc, cv::Mat twc);
    
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    
    PointCloud::Ptr globalMap;
    shared_ptr<thread>  viewerThread;   
    
    bool    shutDownFlag    =false;
    mutex   shutDownMutex;  
    
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
    
    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;
    
    double resolution = 0.01;
    pcl::VoxelGrid<PointT>  voxel;
    pcl::StatisticalOutlierRemoval<PointT> sor; // 创建滤波器对象

    // 观察器
    mutex generation;
    vector<cv::Mat> rgbs;
    vector<cv::Mat> depthes;
    vector<KeyFrame *> kfs;

    PointCloud::Ptr globalMapS;

    Eigen::Vector3d kfplace;
    // 使用PCLVisualizer对象来显示地图
    // pcl::visualization::PCLVisualizer showcamera;
    Eigen::Vector3d currentcamera;
    cv::Mat currentTwc;

};

#endif // POINTCLOUDMAPPING_H
