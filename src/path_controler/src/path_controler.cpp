#include "path_controler.h"
#define STEP 5

const Eigen::Vector3d S_command = Eigen::Vector3d(0.00, 0.00, 0.0);
const Eigen::Vector3d W_command = Eigen::Vector3d(0.07, 0.00, 0.0);
const Eigen::Vector3d A_command = Eigen::Vector3d(0.00, -0.04, 0.0);
const Eigen::Vector3d D_command = Eigen::Vector3d(0.00, 0.04, 0.0);
const Eigen::Vector3d Z_command = Eigen::Vector3d(0.00, 0.00, 6.0);
const Eigen::Vector3d C_command = Eigen::Vector3d(0.00, 0.00, -6.0);

std::vector<Eigen::Vector3d> display_plan;

bool reachGoal = 1;
bool walkFinish = 1;
double stepDegree[5] = {0};
double stepDistance[5] = {0};

int countCommand = 0;
bool FLAG = false;
int holdOn = 0;
geometry_msgs::Pose start_pose_;

void *command_thread(void *ptr)
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        // std::cout<< "main"<<std::endl;
        loop_rate.sleep();
    }
}

void path_controlerCallback(const std_msgs::Bool::ConstPtr &req)
{
    if (reachGoal && walkFinish)
        return;

    std_msgs::Float64MultiArray gaitComm;
    gaitComm.data.resize(3);
    if (req->data == 1)
    {
        cout << "req command" << endl;
        //walkFinish=1;
        //return;
        if (countCommand < STEP)
        {
            cout << "countCommand: " << countCommand << endl;
            cout << "holdOn: " << holdOn << endl;
            if ((!walkFinish) && (!reachGoal))
            {
                if (holdOn == 0 && FLAG == false)
                {
                    if (fabs(stepDegree[countCommand]) > 40)
                        holdOn = fabs(stepDegree[countCommand]) / 8;
                    FLAG = true;
                }

                if (holdOn > 0)
                {
                    cout << "seulement tourner " << stepDegree[countCommand] << endl;

                    for (int j = 0; j < 3; j++)
                    {

                        if (stepDegree[countCommand] > 0)
                            gaitComm.data[j] = Z_command[j];
                        else
                            gaitComm.data[j] = C_command[j];
                    }
                    path_controler_pub.publish(gaitComm);
                    holdOn--;
                    if (holdOn == 0)
                        countCommand = 5;
                }
                else if (holdOn == 0 && stepDistance[countCommand] >= 0.1)
                {

                    for (int j = 0; j < 3; j++)
                    {
                        gaitComm.data[j] = W_command[j];
                    }

                    path_controler_pub.publish(gaitComm);
                    countCommand++;
                    FLAG = false;
                }
            }
        }
        else
        {
            cout << "sleeping" << endl;
            sleep(6);
            FLAG = false;
            countCommand = 0;
            walkFinish = 1;
        }
    }
}

void pathCallback(const nav_msgs::Path &path)
{
    //cout<<"walkedFinish: "<<walkFinish<<endl;
    //cout<<"reachGoal: "<<reachGoal<<endl;
    cout << "path comming" << endl;
    if (!walkFinish)
        return;
    reachGoal = (path.poses.size() == 1) ? 1 : 0;
    if (reachGoal)
        return;
    cout << "path analysis" << endl;

    double initX = path.poses[0].pose.position.x;
    double initY = path.poses[0].pose.position.y;
    // float rotaDegree = 57.29*atan2(2*path.poses[0].pose.orientation.z*path.poses[0].pose.orientation.w,1-2*path.poses[0].pose.orientation.z*path.poses[0].pose.orientation.z);
    cout << 57.29 * path.poses[0].pose.orientation.x << endl;
    cout << 57.29 * path.poses[0].pose.orientation.y << endl;
    cout << 57.29 * path.poses[0].pose.orientation.z << endl;
    cout << 57.29 * path.poses[0].pose.orientation.w << endl;
    cout << 57.29 * start_pose_.orientation.x << endl;
    cout << 57.29 * start_pose_.orientation.y << endl;
    cout << 57.29 * start_pose_.orientation.z << endl;
    cout << 57.29 * start_pose_.orientation.w << endl;
    double q0 = start_pose_.orientation.w;
    double q1 = start_pose_.orientation.x;
    double q2 = start_pose_.orientation.y;
    double q3 = start_pose_.orientation.z;
    double rotaDegree = 57.29 * atan2(2*(q0*q3+q1*q2), 1- 2*(q3*q3 + q2*q2));
    // double rotaDegree = 57.29 * atan2(2 * q2 * q3, 1 - 2 * q2 * q2);
    //double initX = start_pose_.position.x;
    //double initY = start_pose_.position.y;
    //float rotaDegree = 57.29*atan2(2*start_pose_.orientation.z*start_pose_.orientation.w,1-2*start_pose_.orientation.z*start_pose_.orientation.z);
    //cout<<"init pos:("<<initXqq<<","<<initYqq<<") rota:"<<rotaDegreeqq<<endl;
    cout << "init pos:(" << initX << "," << initY << ") rota:" << rotaDegree << endl;

    double tmpX1, tmpY1;
    float distance, goalRota, goalRotaDegree, deltaDegree;
    int stepCounter = 0;

    for (int i = 0; i < 5; i++)
        stepDistance[i] = stepDegree[i] = 0;

    cout << "Solution size:" << path.poses.size() << endl;

    for (int i = 1; i < path.poses.size() - 1; i++)
    {
        tmpX1 = path.poses[i].pose.position.x;
        tmpY1 = path.poses[i].pose.position.y;

        distance = sqrt((tmpX1 - initX) * (tmpX1 - initX) + (tmpY1 - initY) * (tmpY1 - initY));
        goalRota = atan2(tmpY1 - initY, tmpX1 - initX);
        goalRotaDegree = (float)(goalRota * 57.29);
        deltaDegree = goalRotaDegree - rotaDegree;

        if (deltaDegree < -180)
        {
            deltaDegree += 360;
        }
        else if (deltaDegree > 180)
        {
            deltaDegree -= 360;
        }
        //cout<<"tmp pos:("<<tmpX1<<","<<tmpY1<<") distance:"<<distance<<" deltaDegree:"<<deltaDegree<<" goalDegree:"<<goalRotaDegree<<endl;

        if (stepCounter < 5 && (distance > 0.11))
        {
            cout << "add step -> ";
            cout << "distance: " << distance << ", "
                 << "deltaDegree: " << deltaDegree << endl;
            stepDistance[stepCounter] = distance;
            stepDegree[stepCounter] = deltaDegree;
            if (++stepCounter == 5)
                break;
            initX = tmpX1;
            initY = tmpY1;
            rotaDegree = goalRotaDegree;
        }
    }
    walkFinish = 0;
}

void startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start_pose)
{
    // set start:
    start_pose_ = start_pose->pose.pose;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "path_controler");
    ros::NodeHandle nh;

    path_sub = nh.subscribe("path", 1, &pathCallback);
    req_path_controler_sub = nh.subscribe<std_msgs::Bool>("requestGaitCommand", 1, &path_controlerCallback);
    start_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &startCallback);

    path_controler_pub = nh.advertise<std_msgs::Float64MultiArray>("gaitCommand", 1);

    pthread_t thread_command;
    pthread_create(&thread_command, NULL, command_thread, NULL);

    //ros::spin();
    while (ros::ok())
    {
        sleep(6);
        ros::spinOnce();
    }
}
