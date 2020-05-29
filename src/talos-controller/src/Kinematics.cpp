/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Kinematics.h"

using namespace KDL;


using namespace GaitManager;

const double Kinematics::TorsoHight = 0.25;

const double Kinematics::xTorsoToHip = 0.025;
const double Kinematics::yTorsoToHip = 0.037;
const double Kinematics::zTorsoToHip = -0.08;

const double Kinematics::zHipToKnee = -0.093;
const double Kinematics::zKneeToAnkle  = -0.093;
const double Kinematics::zAnkleToSole = -0.035;
const double Kinematics::yAnkleToSole = 0.008;

Kinematics* Kinematics::m_UniqueInstance = new Kinematics();

Kinematics::Kinematics() {
    LeftLeg.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(xTorsoToHip, yTorsoToHip, zTorsoToHip))));
    LeftLeg.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.0))));
    LeftLeg.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0, zHipToKnee))));
    LeftLeg.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0, zKneeToAnkle))));
    LeftLeg.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.0))));
    LeftLeg.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, yAnkleToSole, zAnkleToSole))));

    RightLeg.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(xTorsoToHip, -yTorsoToHip, zTorsoToHip))));
    RightLeg.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.0))));
    RightLeg.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,zHipToKnee))));
    RightLeg.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,zKneeToAnkle))));
    RightLeg.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.0))));
    RightLeg.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,-yAnkleToSole,zAnkleToSole))));

}

Kinematics::~Kinematics() {
}

void Kinematics::ForwardKinematics(double *jointvalue) {
    double angle[21];
    int mirror[21] = {0,     /*arm*/1, 1, 1, 1, 1, 1,      /*leg*/-1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1,     /*head*/1, 1};
    for(int i=1; i<21; i++) {
        angle[i] = mirror[i]*(jointvalue[i]-2048.0)/4096.0*2*3.1415926;
    }

    unsigned int nj = LeftLeg.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(LeftLeg);

    jointpositions(0)=angle[8];
    jointpositions(1)=angle[10];
    jointpositions(2)=angle[12];
    jointpositions(3)=angle[14];
    jointpositions(4)=angle[16];
    jointpositions(5)=angle[18];

    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);

    if(kinematics_status>=0) {
        // std::cout<<"read  "<<cartpos.p(1)<<std::endl;
        //std::cout << cartpos <<std::endl;
        //printf("%s \n","Succes, thanks KDL!");
    } else {
        //printf("%s \n","Error: could not calculate forward kinematics :(");
    }

}