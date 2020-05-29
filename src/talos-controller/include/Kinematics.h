#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

namespace GaitManager
{
class Kinematics
{
private:
    static Kinematics* m_UniqueInstance;
    Kinematics();
protected:

public:
    static const double TorsoHight;
    static const double xTorsoToHip;
    static const double yTorsoToHip;
    static const double zTorsoToHip;
    static const double zHipToKnee;
    static const double zKneeToAnkle;
    static const double zAnkleToSole;
    static const double yAnkleToSole;

    ~Kinematics();
    static Kinematics* GetInstance()            {
        return m_UniqueInstance;
    }

    KDL::Chain LeftLeg, RightLeg;
    KDL::JntArray jointpositions;
    KDL::Frame cartpos;
    bool kinematics_status;

    void ForwardKinematics(double *jointvalue);

};
}

#endif
