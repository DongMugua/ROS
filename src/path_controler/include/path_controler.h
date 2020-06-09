#include <iostream> 
#include <pthread.h>


#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>  
#include <math.h>  
using namespace std;

int countCommand=20;

ros::Publisher path_controler_pub;
ros::Subscriber req_path_controler_sub;
ros::Subscriber path_sub;
