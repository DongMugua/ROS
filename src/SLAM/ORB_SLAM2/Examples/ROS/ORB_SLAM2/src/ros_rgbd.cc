/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include "../../../include/System.h"
#include "../../../include/Converter.h"
#include "../../../include/Observer.h"

using namespace std;

class ImageGrabber
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM)
    {
        pclpub = make_shared<thread>(bind(&ImageGrabber::pclpublish, this));
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    void PublishPose(cv::Mat Tcw);
    void pclpublish();
    ORB_SLAM2::System *mpSLAM;
    ros::Publisher *pPosPub;

    shared_ptr<thread> pclpub;
};

//ros::Publisher pPosPub;

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    static tf::TransformBroadcaster mTfBr;
    geometry_msgs::PoseWithCovarianceStamped poseMSG;

    if (!Tcw.empty())
    {
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
        Eigen::Matrix<double, 3, 3> eigenRwc = ORB_SLAM2::Converter::toMatrix3d(Rwc);
        Eigen::Vector3d angles = eigenRwc.eulerAngles(2, 0, 1);

        tf::Quaternion Q;
        Q.setRPY(angles[0], -angles[1], -angles[2]);
        tf::Vector3 V(twc.at<float>(2), -twc.at<float>(0), -twc.at<float>(1));
        tf::Transform tfTcw(Q, V);
        mTfBr.sendTransform(tf::StampedTransform(tfTcw, ros::Time::now(), "map", "base_link"));

        cout << "rawx: " << setprecision(4) << twc.at<float>(0) << setprecision(4) << " rawy: " << setprecision(4) << twc.at<float>(1) << " rawz: " << twc.at<float>(2) << endl;
        cout << "newx: " << setprecision(4) << V.x() << setprecision(4) << " newy: " << setprecision(4) << V.y() << " newz: " << V.z() << endl;
        // poseMSG.header.
        poseMSG.pose.pose.position.x = V.x();
        poseMSG.pose.pose.position.y = V.y();
        poseMSG.pose.pose.position.z = V.z();
        poseMSG.pose.pose.orientation.x = Q.x();
        poseMSG.pose.pose.orientation.y = Q.y();
        poseMSG.pose.pose.orientation.z = Q.z();
        poseMSG.pose.pose.orientation.w = Q.w();
        poseMSG.header.frame_id = "map";
        poseMSG.header.stamp = ros::Time::now();
        (pPosPub)->publish(poseMSG);
    }
}

void ImageGrabber::pclpublish()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/pcltext", 1);
    string frameid = "camera";
    sensor_msgs::PointCloud2 msg;
    while (1)
    {
        PointCloud::Ptr pointmap((mpSLAM->getMyPclMapping())->Getglobalmap());
        pcl::PointCloud<pcl::PointXYZ> pclmsg;

        //octomap_server 转换需要的是没有颜色的点云地图
        //所以将以构建的彩色点云地图变成无色的
        for (int i = 0; i < pointmap->points.size(); i++)
        {
            pcl::PointXYZ point;
            point.x = pointmap->points[i].x;
            point.y = pointmap->points[i].y;
            point.z = pointmap->points[i].z;
            pclmsg.points.push_back(point);
        }

        pcl::toROSMsg(pclmsg, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frameid;
        // 发布地图信息
        pub.publish(msg);
        usleep(2000);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 5)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings use_viewer reuse_map" << endl;
        ros::shutdown();
        return 1;
    }
    bool bUseViewer = true;
    if (!strcmp(argv[3], "false"))
    {
        bUseViewer = false;
    }
    bool bReuseMap = false;
    if (!strcmp(argv[4], "true"))
    {
        bReuseMap = true;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, bUseViewer, bReuseMap);
    // MyPclMapping PclMapping;
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5);
    igb.pPosPub = &(PosPub);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save map
    SLAM.SaveMap("Slam_Map.bin");

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    //if(mpSLAM->mpTracker->mState != Tracking::LOST)
    PublishPose(pose);
}
