#include <cv.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sstream>
#include <fstream>
using namespace cv;
using namespace std;

void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

ofstream rgbfile("./rgb.txt",ios::out);
ofstream depfile("./dep.txt",ios::out);

int main(int argc, char** argv)
{
	ros::init(argc,argv,"imgsaver");
	ros::start();
	ros::NodeHandle nh;
    	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
   	message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    	sync.registerCallback(boost::bind(&GrabRGBD,_1,_2));


	ros::spin();
	rgbfile.close();
	printf("end of image");
}


void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
	ostringstream convert;
	ostringstream convert_d;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
	ros::Time ros_timeStamp;
    try
    {
	std::string rgbname;
        //cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
	cv_ptrRGB = cv_bridge::toCvCopy(msgRGB);
	//convert<<ros::Time::now();
	//ros_timeStamp=ros::Time::now();
	ros_timeStamp=cv_ptrRGB->header.stamp;
	convert<<"./rgb/"<<ros_timeStamp<<".png";
	rgbname=convert.str();
	imwrite(rgbname,cv_ptrRGB->image);
	rgbname=rgbname.erase(0,1);
	rgbfile<<ros_timeStamp<<" "<<rgbname<<endl;
	//std::cout<<rgbname<<std::endl;
	//imwrite("./rgb/hello.jpg",cv_ptrRGB->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        //cv_ptrD = cv_bridge::toCvShare(msgD);
	std::string depname;
	cv_ptrD = cv_bridge::toCvCopy(msgD);
	convert_d<<"./dep/"<<ros_timeStamp<<".png";
	depname=convert_d.str();
	imwrite(depname,cv_ptrD->image);
	depname=depname.erase(0,1);
	depfile<<ros_timeStamp<<" "<<depname<<endl;
	//std::cout<<depname<<std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}

