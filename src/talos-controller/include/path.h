#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

char reachGoal = 1;
char walkFinish = 1;
double stepDegree[5] = {0};
double stepDistance[5] = {0};

void pathCallback(const nav_msgs::Path& path)  {
    cout<<"walked"<<endl;
    if(!walkFinish)
        return;
    reachGoal = (path.poses.size()==1)?1:0;
    if(reachGoal)
        return;

    double initX = path.poses[0].pose.position.x;
    double initY = path.poses[0].pose.position.y;
    float rotaDegree = 57.6*atan2(2*path.poses[0].pose.orientation.z*path.poses[0].pose.orientation.w,1-2*path.poses[0].pose.orientation.z*path.poses[0].pose.orientation.z);
    cout<<"init pos:("<<initX<<","<<initY<<") rota:"<<rotaDegree<<endl;
    
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
        goalRotaDegree=(float)(goalRota*57.6);
        deltaDegree = goalRotaDegree - rotaDegree;
        if(deltaDegree<-180) {
            deltaDegree += 360;
        } else if(deltaDegree>180) {
            deltaDegree -= 360;
        }
        cout<<"tmp pos:("<<tmpX1<<","<<tmpY1<<") distance:"<<distance<<" deltaDegree:"<<deltaDegree<<" goalDegree:"<<goalRotaDegree<<endl;

        if(stepCounter<5 && (distance>0.08)) {
            cout<<"add step"<<endl;
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
