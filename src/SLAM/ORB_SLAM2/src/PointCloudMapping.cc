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

#include "PointCloudMapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "Converter.h"
#include <iostream>
using namespace std;

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;

    this->resolution = 0.02;

    voxel.setLeafSize( resolution, resolution, resolution);

    this->sor.setMeanK(50);                                //设置在进行统计时考虑查询点邻近点数
    this->sor.setStddevMulThresh(1.0);                     //设置判断是否为离群点的阈值

    globalMap = boost::make_shared< PointCloud >( );
    globalMapS = boost::make_shared< PointCloud >( );

}

void PointCloudMapping::shutdown()
{
    size_t N=0;
    {
      unique_lock<mutex> lck( keyframeMutex );
      N = keyframes.size();
    }

    for ( size_t i=0; i<N ; i++ )
    //for ( size_t i=0; i<5 ; i++ )
    {
        if(keyframes[i]->isBad())
        {
	    cout<<"bad bad..."<<endl; 
	    cout<<"skip bad kf..."<<endl;

	    continue;
        }

	cout<<"process"<<i<<endl;	

        PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
        *globalMap += *p;
    }

    PointCloud::Ptr tmp(new PointCloud());
    voxel.setInputCloud( globalMap );
    voxel.filter( *tmp );
    globalMap->swap( *tmp );

    cout<<"show global map, size="<<globalMap->points.size()<<endl;
    if(!globalMap->empty())                                           // save the pointcloud without optimization
    {
        PointCloud::Ptr tmp(new PointCloud());
        sor.setInputCloud(globalMap);            // remove outlier points      
        sor.filter( *tmp );
        globalMap->swap( *tmp );                    
        pcl::io::savePCDFileBinary( "pointcloud.pcd", *globalMap );
        cout<<"Save point cloud file successfully!"<<endl;
    }
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.05 || d > 20)
                continue;
            PointT p;
            //p.z = d;
            //p.x = ( n - kf->cx) * p.z / kf->fx;
            //p.y = ( m - kf->cy) * p.z / kf->fy;
            
            double rawx,rawy,rawz;

            rawz = d;
            rawx = ( n - kf->cx) * d / kf->fx;
            rawy = ( m - kf->cy) * d / kf->fy;

            p.z = -rawy;//d
            p.x =  rawz;//( n - kf->cx) * d / kf->fx;
            p.y = -rawx;//( m - kf->cy) * d / kf->fy;

            p.r = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.b = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    //cout<<T.inverse().matrix()<<endl;
    PointCloud::Ptr cloud(new PointCloud);
    
    cout<<"raw T"<<endl;
    cout<<T.matrix()<<endl;
    cout<<"T inverse"<<endl;
    cout<<T.inverse().matrix()<<endl;
    
    //0.00525885
    Eigen::Matrix<double,3,3> eigRwc = T.inverse().matrix().block(0,0,3,3);
    Eigen::Vector3d rawAngles = eigRwc.eulerAngles(0,1,2);
    Eigen::Vector3d eigAngles = eigRwc.eulerAngles(2,0,1);
    Eigen::Vector3d eigAnglesTrans(eigAngles(0),-eigAngles(1),-eigAngles(2)); 
    
    // RPY to matrix
    Eigen::AngleAxisd rollAngle(eigAnglesTrans(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(eigAnglesTrans(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(eigAnglesTrans(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    //Eigen::Quaternion<double> q = Eigen::Quaternion<double>(eigAnglesTrans);

    Eigen::Matrix3d rotationMatrix = q.matrix();
    
    //pretranslate
    Eigen::Vector3d preTrans(T.inverse().matrix()(0,3),T.inverse().matrix()(1,3),T.inverse().matrix()(2,3));
    Eigen::Vector3d goalTrans(preTrans(2),-preTrans(0),-preTrans(1));
    // transfer martix
    Eigen::Matrix4d desMatrix;
    desMatrix.setIdentity();
    //desMatrix.block(0,0,3,3) = rotationMatrix;
    desMatrix.block<3,3>(0,0) = rotationMatrix;
    desMatrix.block<3,1>(0,3) = goalTrans;

    //cout<<"preTrans : "<<preTrans<<endl;
    //cout<<"goalTrans : "<<goalTrans<<endl;
    //cout<<"rotationMatrix:   "<<rotationMatrix<<endl;
    //cout<<"desMatrix: : "<<endl;
    //cout<<desMatrix<<endl;
    Eigen::Vector3d newAngles = rotationMatrix.eulerAngles(0,1,2);

    //cout<<" rawR "<<rawAngles(0)<<" rawP "<<rawAngles(1)<<" rawY "<<rawAngles(2)<<endl;
    //cout<<" middleR "<<eigAnglesTrans(0)<<" middleP "<<eigAnglesTrans(1)<<" middleY "<<eigAnglesTrans(2)<<endl;
    //cout<<" newR "<<newAngles(0)<<" newP "<<newAngles(1)<<" newY "<<newAngles(2)<<endl;



    //pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    pcl::transformPointCloud( *tmp, *cloud, desMatrix);
    

    cloud->is_dense = false;
    
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


// 观察器

void PointCloudMapping::insertKF(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    //在更新已有地图时加锁，防止和Getglobalmap函数冲突
    unique_lock<mutex> locker(generation);
    cv::Mat rgb = color.clone();
    cv::Mat D = depth.clone();
    //构建关键帧对应的点云地图
    generatepcl(kf, rgb, D);
    //将这些信息保存下来
    rgbs.push_back(color.clone());
    depthes.push_back(depth.clone());
    kfs.push_back(kf);
}

void PointCloudMapping::generatepcl(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    //用temp保存关键帧对应的点云地图
    PointCloud::Ptr temp(new PointCloud());
    //获得关键帧的估计位姿
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPoseInverse());
    //遍历彩色图像，构建点云地图（在关键帧的坐标系下）
    for (int v = 0; v < color.rows; v += 3) //为了降低地图占用的内存大小，遍历的步长定为3
    {
        for (int u = 0; u < color.cols; u += 3)
        {
            float d = depth.ptr<float>(v)[u];
            if (d < 0)
                continue;
            PointT p;
            //计算地图点的坐标
            p.z = d;
            p.x = p.z * (u - kf->cx) / kf->fx;
            p.y = p.z * (v - kf->cy) / kf->fy;
            //给地图点上色
            p.b = color.data[v * color.step + u * color.channels()];
            p.g = color.data[v * color.step + u * color.channels() + 1];
            p.r = color.data[v * color.step + u * color.channels() + 2];
            temp->points.push_back(p);
        }
    }
    PointCloud::Ptr cloud(new PointCloud());
    //将新构建的地图变换到世界坐标系下
    pcl::transformPointCloud(*temp, *cloud, T.matrix());
    cloud->is_dense = false;
    //完成新旧地图的叠加
    (*globalMapS) += (*cloud);
}

//参数：从相机坐标系到世界坐标系的旋转矩阵和平移向量
void PointCloudMapping::setcurrentCamera(cv::Mat Rwc, cv::Mat twc)
{
    cv::Mat Twc = cv::Mat::eye(4, 4, Rwc.type());
    //获得当前相机的位姿
    Rwc.copyTo(Twc(cv::Rect(0, 0, 3, 3)));
    twc.copyTo(Twc(cv::Rect(3, 0, 1, 3)));
    currentTwc = Twc.clone();
    g2o::SE3Quat q = ORB_SLAM2::Converter::toSE3Quat(currentTwc);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(q.rotation());
    T.pretranslate(q.translation());
    Eigen::Vector3d currentpoint(0.1, 0.1, 0.1);
    //获得相机在世界坐标系下的坐标位置
    currentcamera = T * currentpoint;
    // PointCloud::Ptr temp(Getglobalmap()); //获得完整的点云地图
    //刷新PCLVisualizer对象中保存的内容
    // showcamera.removeAllPointClouds();
    // showcamera.removeAllShapes();
    // showcamera.addPointCloud(temp);
    // Eigen::Quaternionf qf(float(q.rotation().coeffs()[3]),
    //                       float(q.rotation().coeffs()[0]), float(q.rotation().coeffs()[1]),
    //                       float(q.rotation().coeffs()[2]));
    //用一个立方体来表示相机
    // showcamera.addCube(Eigen::Vector3f(q.translation()[0], q.translation()[1], q.translation()[2]), qf, 0.7, 0.7, 1.2);
    //显示一次当前地图中的内容
    // showcamera.spinOnce();
}