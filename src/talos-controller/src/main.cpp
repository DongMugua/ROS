#include "main.h"
#include <curses.h>
#include "Kinematics.h"

#include <signal.h>
#include <time.h>

using namespace GaitManager ;
using namespace std;
using namespace boost;
using namespace KDL;

struct timeval timeReal;
double start_time;

void timerCallback(int in)
{
    gettimeofday(&timeReal,NULL);
    if(fabs(timeReal.tv_usec-start_time-20000) > 2000)
        printf("time %lf\n",timeReal.tv_usec-start_time);
    start_time = timeReal.tv_usec;
    mWalk.run();
    GyroBalance();//该修正需位于run之后
    IdealToControl();
    cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);
    if(mWalk.StepCount ==mWalk.StepCountTarget) {
        mWalk.stopWalking();
    }
}

void initTimer()
{
    timer_t timer;
    struct sigevent evp;
    struct itimerspec ts;
    memset(&evp, 0, sizeof(evp));
    evp.sigev_notify = SIGEV_SIGNAL;
    evp.sigev_signo = SIGUSR1;
    signal(SIGUSR1, timerCallback);
    timer_create(CLOCK_REALTIME, &evp, &timer);
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 20000000;
    ts.it_value.tv_sec = 0;
    ts.it_value.tv_nsec = 10;
    timer_settime(timer, 0, &ts, NULL);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ros_node");
    advertise();

    ros::NodeHandle nh;
    ros::Subscriber sub2 = nh.subscribe("path",1, &pathCallback);

    if(cm730.Connect() == true)
    {
        Scan(&cm730);
        Parainit();
        stand();
        //pthread_t thread_t;
        //pthread_create(&thread_t, NULL, JY901_thread, NULL);

        gettimeofday(&timeReal,NULL);
        start_time = timeReal.tv_usec;
        initTimer();
        initInput();
        double *angle;
        while(true) {
            switch(readInput(1)) {
            case 't':
                while(true)
                {
                    sleep(2);
                    if (readInput(1) == 'q')
                        break;
                    ros::spinOnce();
                    autoWalk();
                }
                break;
            case 'q':
                exit(0);
                break;
            case 'i':
                mWalk.JointValue[20] -= 20;//上   上有范围限制
                if(mWalk.JointValue[20]<2400) {
                    mWalk.JointValue[20] = 2400;
                    printf("%f",mWalk.JointValue[20]);

                }
                break;
            case 'k':
                mWalk.JointValue[20] += 20;//下
                break;
            case 'j':
                mWalk.JointValue[19] -= 20;//左
                break;
            case 'l':
                mWalk.JointValue[19] += 20;//右
                break;
            case 'w':
                angle = new double[5] {0};
                mWalk.start(0.04,5,angle);
                break;
            case 's':
                angle = new double[5] {0};
                mWalk.start(-0.04,5,angle);
                break;
            case 'a':
                angle = new double[5] {0,15,15,15,15};
                mWalk.start(0.0,5,angle);
                break;
            case 'd':
                angle = new double[5] {0,-15,-15,-15,-15};
                mWalk.start(0.0,5,angle);
                break;
            case '0':
                mWalk.Y_sidewalk = -0.03;
                mWalk.Step_TC_y = 6.5;
                mWalk.StepCountTarget = 5;
                mWalk.PatternInit_side();
                mWalk.RobotState=mWalk.Walk_start_side;
                break;
            case '9':
                mWalk.Y_sidewalk = 0.03;
                mWalk.Step_TC_y = 6.5;
                mWalk.StepCountTarget = 5;
                mWalk.PatternInit_side();
                mWalk.RobotState=mWalk.Walk_start_side;
                break;
            default:
                break;
            }
        }
    }
    destoryInput();
    return 0;
}

void LIPM_publish()
{
    std_msgs::Float64 msg_val;
    // msg_val.data=int(mWalk.CoM_y_RelaToP[mWalk.StepRhythm]*1000);
    // com_y_c.publish(msg_val);

    // int temp;
    // if(mWalk.StepRhythm == 1)
    //  odometer_y -=0.12;

    if(mWalk.StepRhythm != 0) {
        msg_val.data=int((mWalk.CoM_y_RelaToP[mWalk.StepRhythm]+odometer_y)*1000);
        com_y_c.publish(msg_val);
    }
    else {
        msg_val.data=int((mWalk.CoM_y_RelaToP[mWalk.RhythmCount]+odometer_y)*1000);
        com_y_c.publish(msg_val);
    }
}

void advertise()
{
    ros::NodeHandle n;
    error_x =n.advertise<std_msgs::Float64>("error_x",1000);
    error_y =n.advertise<std_msgs::Float64>("error_y",1000);
    v_x_c =n.advertise<std_msgs::Float64>("v_x_c",1000);
    v_x_m =n.advertise<std_msgs::Float64>("v_x_m",1000);

    com_x_m =n.advertise<std_msgs::Float64>("com_x_m",1000);
    com_x_c =n.advertise<std_msgs::Float64>("com_x_c",1000);
    com_x_i =n.advertise<std_msgs::Float64>("com_x_i",1000);
    com_y_m =n.advertise<std_msgs::Float64>("com_y_m",1000);
    com_y_c =n.advertise<std_msgs::Float64>("com_y_c",1000);

    JY901X = n.advertise<std_msgs::Float64>("JY901X",1000);
    JY901Y = n.advertise<std_msgs::Float64>("JY901Y",1000);

    torsoroll = n.advertise<std_msgs::Float64>("torsoroll",1000);
    torsopitch = n.advertise<std_msgs::Float64>("torsopitch",1000);

    StepRhythm = n.advertise<std_msgs::Float64>("StepRhythm",1000);

    stepOK = n.advertise<std_msgs::Bool>("stepOK",1000);

}

void CoM_measure()
{
    for(id=9; id<=18; id++)
    {
        if(cm730.ReadWord(id, 36, &measuredvalue[id], 0) == CM730::SUCCESS)
        {
            mWalk.JointReadOK=true;
            error[id] = idealvalue[id] - measuredvalue[id];
        }
        // else
        // {mWalk.JointReadOK=false;  printf("id  =  %d    read motor failed!!!!!!!!!\n", id);}
    }

    double length;
    for(int i = 9; i<=18; i++)
        MeasuredAngle[i] = (measuredvalue[i] - JointOffset[i] - 2048)/AngleToValue;
    MeasuredAngle[9] = -MeasuredAngle[9];
    MeasuredAngle[10] = -MeasuredAngle[10];
    MeasuredAngle[13] = -MeasuredAngle[13];
    MeasuredAngle[14] = -MeasuredAngle[14];
    MeasuredAngle[17] = -MeasuredAngle[17];
    MeasuredAngle[18] = -MeasuredAngle[18];

    // CoM_x_caled = mWalk.CoM_x_RelaToP[mWalk.StepRhythm];
    CoM_x_caled = mWalk.CoM_x_RelaToW[mWalk.MeasureLagCircle];
    CoM_y_caled = mWalk.CoM_y_RelaToW[mWalk.MeasureLagCircle];
    CoM_H_caled = 0.176;
    CoM_x_m_old = CoM_x_m;

    // mWalk.StepState = mWalk.Lfoot_stance;
    if(mWalk.StepState == mWalk.Rfoot_stance)
    {
        length = Shank*cos(MeasuredAngle[R_ANKLE_PITCH]/Rad2Deg) + Thigh*cos(( -MeasuredAngle[R_KNEE] - MeasuredAngle[R_ANKLE_PITCH])/Rad2Deg);
        CoM_H_m = length*cos(MeasuredAngle[R_ANKLE_ROLL]/Rad2Deg);
        CoM_x_m =  length*sin(MeasuredAngle[R_ANKLE_ROLL]/Rad2Deg);
        CoM_y_m = Shank*sin(MeasuredAngle[R_ANKLE_PITCH]/Rad2Deg) - Thigh*sin(( -MeasuredAngle[R_KNEE] - MeasuredAngle[R_ANKLE_PITCH])/Rad2Deg);
        CoM_H_m = CoM_H_m + IKoffset_R_H;
        CoM_x_m = CoM_x_m + IKoffset_R_x;
        CoM_y_m = CoM_y_m + IKoffset_R_y;
        torsoRoll = MeasuredAngle[R_ANKLE_ROLL] - MeasuredAngle[R_HIP_ROLL];
        torsoPitch = MeasuredAngle[R_ANKLE_PITCH] - (-MeasuredAngle[R_KNEE] -(-MeasuredAngle[R_HIP_PITCH]));
    }

    else if(mWalk.StepState == mWalk.Lfoot_stance)
    {
        length = Shank*cos(MeasuredAngle[L_ANKLE_PITCH]/Rad2Deg) + Thigh*cos(( -MeasuredAngle[L_KNEE] - MeasuredAngle[L_ANKLE_PITCH])/Rad2Deg);
        CoM_H_m = length*cos(MeasuredAngle[L_ANKLE_ROLL]/Rad2Deg);
        CoM_x_m =  length*sin(MeasuredAngle[L_ANKLE_ROLL]/Rad2Deg);
        CoM_y_m = Shank*sin(MeasuredAngle[L_ANKLE_PITCH]/Rad2Deg) - Thigh*sin(( -MeasuredAngle[L_KNEE] - MeasuredAngle[L_ANKLE_PITCH])/Rad2Deg);
        CoM_y_m = - CoM_y_m;
        CoM_H_m = CoM_H_m + IKoffset_L_H;
        CoM_x_m = CoM_x_m + IKoffset_L_x;
        CoM_y_m = CoM_y_m + IKoffset_L_y;
        torsoRoll = MeasuredAngle[L_ANKLE_ROLL] - MeasuredAngle[L_HIP_ROLL];
        torsoPitch = -MeasuredAngle[L_ANKLE_PITCH] - (MeasuredAngle[L_KNEE] - MeasuredAngle[L_HIP_PITCH]);
    }
    mWalk.CoM_H_measured = CoM_H_m;
    mWalk.CoM_x_measured = CoM_x_m;
    mWalk.CoM_y_measured = CoM_y_m;
}

void GyroBalance()
{
    cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_Y_L, &gyroY, 0);

    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    int dir[14]          = {   -1,        -1,          1,         1,         -1,            1,          -1,        -1,         -1,         -1,         1,            1,           1,           -1      };

    // 侧向为X 正向为Y
    //调试 乘2处默认为乘4
    // mWalk.JointValue[R_HIP_ROLL] += (int)(dir[1] * (gyroX-512) * BALANCE_HIP_ROLL_GAIN*4); // R_HIP_ROLL
    mWalk.JointValue[R_KNEE] -= (int)(dir[3] * (gyroY-512) * BALANCE_KNEE_GAIN*2); // R_KNEE
    mWalk.JointValue[R_ANKLE_PITCH] -= (int)(dir[4] * (gyroY-512) * BALANCE_ANKLE_PITCH_GAIN*2); // R_ANKLE_PITCH
    // mWalk.JointValue[R_ANKLE_ROLL] -= (int)(dir[5] * (gyroX-512) * BALANCE_ANKLE_ROLL_GAIN*4); // R_ANKLE_ROLL

    // mWalk.JointValue[L_HIP_ROLL] += (int)(dir[7] * (gyroX-512) * BALANCE_HIP_ROLL_GAIN*4); // L_HIP_ROLL
    mWalk.JointValue[L_KNEE] -= (int)(dir[9] * (gyroY-512) * BALANCE_KNEE_GAIN*2); // L_KNEE
    mWalk.JointValue[L_ANKLE_PITCH] -= (int)(dir[10] * (gyroY-512) * BALANCE_ANKLE_PITCH_GAIN*2); // L_ANKLE_PITCH
    // mWalk.JointValue[L_ANKLE_ROLL] -= (int)(dir[11] * (gyroX-512) * BALANCE_ANKLE_ROLL_GAIN*4); // L_ANKLE_ROLL
}

void IdealToControl()
{
    for(id=1; id<=20; id++)
    {
        int n = id*5-5;
        int temp;

        //速度控制，动作会更顺滑，受负载时的偏差会更大 对步态影响不好
        // speedvalue = (mWalk.JointValue[id] - idealvalue[id]) / (mWalk.mTimeStep*0.001) *0.1285;//速度控制
        // speedvalue = fabs(speedvalue);
        // if(speedvalue >1023)
        //  printf("overspeed   id = %d   speedvalue  = %d\n",id ,  speedvalue);
        // if(speedvalue >1023) //调试  1023是否为速度上限需要测试
        //  speedvalue = 1023;
        // param[n+3]=speedvalue & 0xff;
        // temp = speedvalue & 0xff00;
        // param[n+4]=temp>> 8;

        idealvalue[id] = mWalk.JointValue[id] + JointOffset[id] + ControlOffset[id];
        // controlvalue[id] = idealvalue[id] + error[id]/2;
        controlvalue[id] = idealvalue[id];
        param[n]=id;
        param[n+1]=controlvalue[id] & 0xff;
        temp = controlvalue[id] & 0xff00;
        param[n+2]=temp>> 8;
    }
}

void readerror()
{
    for(int i=0; i<21; i++)   error[i] = 0;
    // if(mWalk.StepState == mWalk.Rfoot_stance)
    {
        for(id=9; id<=18; id+=2)
        {
            if(cm730.ReadWord(id, 36, &measuredvalue[id], 0) == CM730::SUCCESS)
            {
                mWalk.JointReadOK=true;
                error[id] = idealvalue[id] - measuredvalue[id];
            }
            // else
            // {mWalk.JointReadOK=false;  printf("id  =  %d    read motor failed!!!!!!!!!\n", id);}
        }
    }
    // else if(mWalk.StepState == mWalk.Lfoot_stance)
    {
        for(id=10; id<=18; id+=2)
        {
            if(cm730.ReadWord(id, 36, &measuredvalue[id], 0) == CM730::SUCCESS)
            {
                mWalk.JointReadOK=true;
                error[id] = idealvalue[id] - measuredvalue[id];
            }
            // else
            // {mWalk.JointReadOK=false;  printf("id  =  %d    read motor failed!!!!!!!!!     measured failed !!!!!!!!!!!!\n", id);}
        }
    }
}

void stand()
{
    //初始化param
    int n=0;
    for(id=1; id<=20; id++) {
        param[n++] = id;
        param[n++] = CM730::GetLowByte(2048);
        param[n++] = CM730::GetHighByte(2048);
        param[n++] = CM730::GetLowByte(800);
        param[n++] = CM730::GetHighByte(800);
    }

    // mWalk.RobotState = mWalk.Walk_stand;
    // mWalk.run();
    mWalk.JointValue[19] = 1900;
    mWalk.JointValue[20] = 2720;

    for(id=1; id<=20; id++)
    {
        int n = id*5-5;
        int value = mWalk.JointValue[id] + JointOffset[id] + ControlOffset[id];
        param[n]=id;
        param[n+1]=value & 0xff;
        int temp = value & 0xff00;
        param[n+2]=temp>> 8;
    }
    cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);
    sleep(1);
}

void Parainit()
{
    id = 9;
    mJOINT.SetPGain(id,150);
    mJOINT.SetIGain(id,140);
    mJOINT.SetDGain(id,90);

    id = 10;
    mJOINT.SetPGain(id,150);
    mJOINT.SetIGain(id,140);
    mJOINT.SetDGain(id,90);

    for(int i=1; i<21; i++)
    {
        cm730.WriteByte(i,MX28::P_P_GAIN,mJOINT.GetPGain(i),0);
        cm730.WriteByte(i,MX28::P_I_GAIN,mJOINT.GetIGain(i),0);
        cm730.WriteByte(i,MX28::P_D_GAIN,mJOINT.GetDGain(i),0);
    }

    for(id=1; id<=20; id++)
    {
        int n = id*2-2;
        int value = 50;
        param[n]=id;
        param[n+1]=value;
    }
    cm730.SyncWrite(MX28::P_RETURN_DELAY_TIME, 2, JointData::NUMBER_OF_JOINTS - 1, param);
    for(id=1; id<=20; id++)
    {
        int n = id*2-2;
        int value = 1;
        param[n]=id;
        param[n+1]=value;
    }
    cm730.SyncWrite(MX28::P_RETURN_LEVEL, 2, JointData::NUMBER_OF_JOINTS - 1, param);
}

void CopeSerialData(unsigned char ucData)
{
    static unsigned char ucRxBuffer[250];
    static unsigned char ucRxCnt = 0;

    ucRxBuffer[ucRxCnt++]=ucData;
    if (ucRxBuffer[0]!=0x55)
    {
        ucRxCnt=0;
        return;
    }
    if (ucRxCnt<11) {
        return;
    }
    else
    {
        switch(ucRxBuffer[1])
        {
        case 0x50:
            memcpy(&stcTime,&ucRxBuffer[2],8);
            break;
        case 0x51:
            memcpy(&stcAcc,&ucRxBuffer[2],8);
            break;
        case 0x52:
            memcpy(&stcGyro,&ucRxBuffer[2],8);
            break;
        case 0x53:
            memcpy(&stcAngle,&ucRxBuffer[2],8);
            break;
        case 0x54:
            memcpy(&stcMag,&ucRxBuffer[2],8);
            break;
        case 0x55:
            memcpy(&stcDStatus,&ucRxBuffer[2],8);
            break;
        case 0x56:
            memcpy(&stcPress,&ucRxBuffer[2],8);
            break;
        case 0x57:
            memcpy(&stcLonLat,&ucRxBuffer[2],8);
            break;
        case 0x58:
            memcpy(&stcGPSV,&ucRxBuffer[2],8);
            break;
        }
        ucRxCnt=0;
        JY901ReceiveOK = true;
    }
}
