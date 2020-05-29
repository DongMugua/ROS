#include "main_var.h"
#include "path.h"

void autoWalk(){
    if(reachGoal && walkFinish)
        cout<<"Reach goal"<<endl;

    if((walkFinish == 0)&&(!reachGoal)) {
        walkFinish = 1;

    //此时原地转弯
    if(stepDegree[0]>36) {
        double angle[6] = {0, 15, 15,15 , 0 , 0};
        mWalk.start(0.0, 6, angle);

        while(mWalk.RobotState !=mWalk.Walk_standed)
            usleep(100000);
        return;
    } else if(stepDegree[0] < -36) {
        double angle[6] = {0, -15, -15,-15 , 0 , 0};
        mWalk.start(0.0, 6, angle);

        while(mWalk.RobotState !=mWalk.Walk_standed)
            usleep(100000);
        return;
    }

    //此时把第一步的转弯角度分配到后面，会造成走路的偏差
    //当WalkCount小于5时，即到达终点
    if(fabs(stepDegree[0])>15&&fabs(stepDegree[0])<36) {
        stepDegree[0] = stepDegree[0]/3;
        stepDegree[1] = stepDegree[1]+stepDegree[0]/6;
        stepDegree[2] = stepDegree[2]+stepDegree[0]/6;
        stepDegree[3] = stepDegree[3]+stepDegree[0]/6;
        stepDegree[4] = stepDegree[4]+stepDegree[0]/6;
    }
    int WalkCount = 0;
    for(int i =0; i<5; i++) {
        if(stepDistance[i]>0.05)
            WalkCount++;
    }
    //行走步数不能为1
    if(WalkCount<2)
        WalkCount=2;

    /*                  cout<<"degree changed"<<endl;
                        for(int i =0;i<5;i++)
                            cout<<"length:"<<stepDistance[i]<<" degree:"<<stepDegree[i]<<endl;
                        cout<<endl;
    */
    mWalk.start(0.04/*0.05的话步子会太大*/, WalkCount, stepDegree);

    while(mWalk.RobotState !=mWalk.Walk_standed)
        usleep(100000);
    for(int i=0; i<100; i++)
        mWalk.WayPoint_Yaw[i] = 0;

    }
}
