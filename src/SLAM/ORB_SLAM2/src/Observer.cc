#include "Observer.h"
/*
void MyPclMapping::insertKF(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
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

void MyPclMapping::generatepcl(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
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
    (*globalMap) += (*cloud);
}

//参数：从相机坐标系到世界坐标系的旋转矩阵和平移向量
void MyPclMapping::setcurrentCamera(cv::Mat Rwc, cv::Mat twc)
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
    PointCloud::Ptr temp(Getglobalmap()); //获得完整的点云地图
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
}*/