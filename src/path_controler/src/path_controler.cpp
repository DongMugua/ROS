#include "path_controler.h"

const Eigen::Vector3d S_command = Eigen::Vector3d(0.00,0.00,0.0);
const Eigen::Vector3d W_command = Eigen::Vector3d(0.07,0.00,0.0);
const Eigen::Vector3d A_command = Eigen::Vector3d(0.00,-0.04,0.0);
const Eigen::Vector3d D_command = Eigen::Vector3d(0.00,0.04,0.0);
const Eigen::Vector3d Z_command = Eigen::Vector3d(0.00,0.00,10.0);
const Eigen::Vector3d C_command = Eigen::Vector3d(0.00,0.00,-10.0);

std::vector<Eigen::Vector3d> display_plan;

bool reachGoal = 1;
bool walkFinish = 1;
double stepDegree[5] = {0};
double stepDistance[5] = {0};

// void* command_thread(void* ptr)  
// {
//   	ros::Rate loop_rate(10);
// 	while(ros::ok())
// 	{
// 		ros::spinOnce();
// 		// std::cout<< "main"<<std::endl;
		
//     	loop_rate.sleep();
// 	}
// }
int stepTotal = 5;
int countCommand = 0;
bool holdOn;
void path_controlerCallback(const std_msgs::Bool::ConstPtr& req)
{
	if(reachGoal && walkFinish)
		return;
	
	std_msgs::Float64MultiArray gaitComm;
	gaitComm.data.resize(3);
	cout<<"Path controler Callback begin"<<endl;
	if (req->data == 1) {
        if (countCommand < stepTotal) {
            if((!walkFinish)&&(!reachGoal)) {
                holdOn = fabs(stepDegree[countCommand]) >= 15 ? true : false;

                if (holdOn) {
                    cout<<"seulement tourner "<<stepDegree[countCommand]<<endl;
                    for(int j=0; j<3; j++)
                        gaitComm.data[j] = stepDegree[0] > 0 ? Z_command[j] : C_command[j];
                    path_controler_pub.publish(gaitComm);
                } else if (!holdOn && stepDistance[countCommand] >= 0.09) {
                    for (int j=0; j<3; j++) {
                        gaitComm.data[j] = W_command[j];
                    }
                    path_controler_pub.publish(gaitComm);	
                    countCommand++;			
                }
            }
        } else {
            countCommand = 0;
            walkFinish = 1;
        }
	}
	// countCommand++;
}
void usless(int s) {
// void *command_thread(void *ptr)
// {
// 	ros::Rate loop_rate(10);
// 	while (ros::ok())
// 	{
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
// }
// 	if(req->data == 1){
// 		if(reachGoal && walkFinish)
// 		cout<<"Reach goal"<<endl;

// 		if((walkFinish == 0)&&(!reachGoal)) {
// 			// for(int i=0; i<5; i++) {
// 			// 	const double dx = stepDistance[i] * cos(stepDegree[i] * M_PI / 180);
// 			// 	const double dy = stepDistance[i] * sin(stepDegree[i] * M_PI / 180);
// 			// 	const double theta = -stepDegree[i];
// 			// 	const Eigen::Vector3d cmd = Eigen::Vector3d(dx, dy, theta);
// 			// 	for(int j=0; j<3; j++)
// 			// 		gaitComm.data[j] = cmd [j];
// 			// 	path_controler_pub.publish(gaitComm);
// 			// 	sleep(2);
// 			// }
// 			//  此时原地转弯
// 			// if(stepDegree[0]>36) {
// 			// 	double angle[6] = {0, 15, 15,15 , 0 , 0};
// 			// 	mWalk.start(0.0, 6, angle);

// 			// 	while(mWalk.RobotState !=mWalk.Walk_standed)
// 			// 		usleep(100000);
// 			// 	return;
// 			// } else if(stepDegree[0] < -36) {
// 			// 	double angle[6] = {0, -15, -15,-15 , 0 , 0};
// 			// 	mWalk.start(0.0, 6, angle);

// 			// 	while(mWalk.RobotState !=mWalk.Walk_standed)
// 			// 		usleep(100000);
// 			// 	return;
// 			// }

// 			//此时把第一步的转弯角度分配到后面，会造成走路的偏差
// 			//当WalkCount小于5时，即到达终点

// 			for(int i=0; i<5; i++) {

// 				if (fabs(stepDegree[i]) > 15) {
// 					cout<<"seulement tourner "<<stepDegree[i]<<endl;
// 					for(int j=0; j<3; j++)
// 						gaitComm.data[j] = stepDegree[0] > 0 ? Z_command[j] : C_command[j];
// 					path_controler_pub.publish(gaitComm);
// 					sleep(2);
// 					return;
// 				}
// /*
// 				if(fabs(stepDegree[i+1])>10 && fabs(stepDegree[i+1]) <= 15 && i < 4) {
// 				}
// */
// 				double dx = stepDistance[i] * cos( stepDegree[i] * M_PI / 180 );
// 				double dy = stepDistance[i] * sin( stepDegree[i] * M_PI / 180 );
// 				double theta = -stepDegree[i];
// 				if ( dx > 0.07 ) 
// 					dx = 0.07;
// 				else if ( dx < -0.07 )
// 					dx = -0.07;

// 				if ( dy > 0.04 ) 
// 					dy = 0.04;
// 				else if ( dy < -0.04 )
// 					dy = -0.04;	
// 				if ( theta > 10 )
// 					theta = 10;
// 				else if ( theta < -10)
// 					theta = -10;
					
// 				const Eigen::Vector3d cmd = Eigen::Vector3d(dx, dy, theta);
// 				for(int j=0; j<3; j++)
// 					gaitComm.data[j] = cmd [j];
// 				cout<<"dx:"<<dx<<"dy:"<<dy<<"theta:"<<theta<<endl;
// 				path_controler_pub.publish(gaitComm);
// 				sleep(2);
// 			}
// 			// int WalkCount = 0;
// 			// for(int i =0; i<5; i++) {
// 			// 	if(stepDistance[i]>0.05)
// 			// 		WalkCount++;
// 			// }
// 			// //行走步数不能为1
// 			// if(WalkCount<2)
// 			// 	WalkCount=2;

// 			/*                  cout<<"degree changed"<<endl;
// 								for(int i =0;i<5;i++)
// 									cout<<"length:"<<stepDistance[i]<<" degree:"<<stepDegree[i]<<endl;
// 								cout<<endl;
// 			*/
// 			// mWalk.start(0.04/*0.05的话步子会太大*/, WalkCount, stepDegree);

// 			// while(mWalk.RobotState !=mWalk.Walk_standed)
// 			// 	usleep(100000);
// 			// for(int i=0; i<100; i++)
// 			// 	mWalk.WayPoint_Yaw[i] = 0;
// 			walkFinish = 1;
// 		}
// 	}
// }
}

void pathCallback(const nav_msgs::Path& path) {
    cout<<"walkedFinish: "<<walkFinish<<endl;
    cout<<"reachGoal: "<<reachGoal<<endl;
    if(!walkFinish)
        return;
    reachGoal = (path.poses.size()==1)?1:0;
    if(reachGoal)
        return;

    double initX = path.poses[0].pose.position.x;
    double initY = path.poses[0].pose.position.y;
    float rotaDegree = 57.29*atan2(2*path.poses[0].pose.orientation.z*path.poses[0].pose.orientation.w,1-2*path.poses[0].pose.orientation.z*path.poses[0].pose.orientation.z);
    // cout<<"init pos:("<<initX<<","<<initY<<") rota:"<<rotaDegree<<endl;
    
    double tmpX1, tmpY1;
    float distance, goalRota, goalRotaDegree, deltaDegree;
    int stepCounter = 0;

    for(int i=0; i<5; i++)
        stepDistance[i] = stepDegree[i] = 0;
    
    cout<<"Solution size:"<<path.poses.size()<<endl;

    for(int i=1; i<path.poses.size()-1; i++) {
        tmpX1 = path.poses[i].pose.position.x;
        tmpY1 = path.poses[i].pose.position.y;

        distance = sqrt((tmpX1-initX)*(tmpX1-initX)+(tmpY1-initY)*(tmpY1-initY));
        goalRota = atan2(tmpY1-initY,tmpX1-initX);
        goalRotaDegree=(float)(goalRota*57.29);
        deltaDegree = goalRotaDegree - rotaDegree;
        if(deltaDegree<-180) {
            deltaDegree += 360;
        } else if(deltaDegree>180) {
            deltaDegree -= 360;
        }
        // cout<<"tmp pos:("<<tmpX1<<","<<tmpY1<<") distance:"<<distance<<" deltaDegree:"<<deltaDegree<<" goalDegree:"<<goalRotaDegree<<endl;

        if(stepCounter<5 && (distance>0.08)) {
            cout<<"add step -> ";
            cout<<"distance: "<<distance<<", "<<"deltaDegree: "<<deltaDegree<<endl;
            stepDistance[stepCounter] = distance;
            stepDegree[stepCounter] = deltaDegree;
            if(++stepCounter == 5)
                break;
            initX = tmpX1;
            initY = tmpY1;
            rotaDegree = goalRotaDegree;
        }
    }
    walkFinish = 0;
}


int main(int argc, char** argv)
{	

	ros::init(argc, argv, "path_controler");
	ros::NodeHandle nh;	

    path_sub = nh.subscribe("path",1, &pathCallback);
	req_path_controler_sub = nh.subscribe<std_msgs::Bool>("requestGaitCommand",1,&path_controlerCallback);
	
	path_controler_pub = nh.advertise<std_msgs::Float64MultiArray>("gaitCommand",1);

	// pthread_t thread_command;
	// pthread_create(&thread_command, NULL, command_thread, NULL);

	ros::spin();
    // while(ros::ok())
	// {
	// 	sleep(2);
	// 	ros::spinOnce();
	// }
}
