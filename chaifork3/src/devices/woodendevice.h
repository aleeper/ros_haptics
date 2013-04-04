#ifndef WOODENDEVICE_H
#define WOODENDEVICE_H

#ifndef DOFENUM
#define DOFENUM
enum DOF {
    TURNTABLE = 0,
    ARM1,
    ARM2,
    DOFS
};
#endif
//#include "simulatedwoodendevice.h" // for the DOF defenition
#include "s626da.h"
#include "s626encoder.h"

#include "math/CMaths.h"

class WoodenDevice
{
public:
    static const double capstanDiameter2 = 10.57e-3;
    static const double capstanDiameter1 = 10.57e-3;
    static const double capstanDiameter0 = 10.57e-3; // unmeasured
    static const double turntableDiameter = 160e-3;
    static const double arm1Diameter = 120e-3;
    static const double arm2Diameter = 120e-3;
    static const double dTurntableArm1 = 0.0982; // meter
    static const double dArm1Arm2 = 0.205;
    static const double dArm2Handle = 0.22475;
    WoodenDevice(S626DA **da, S626Encoder **encoder);
    WoodenDevice();

    void setEnable(bool enabled){
        setMotorTorque(DOF(0),0);
        setMotorTorque(DOF(1),0);
        setMotorTorque(DOF(2),0);
        this->enabled = enabled;
    }

    void setMotorTorque(DOF dof, double Nm);
    bool wouldSaturate(DOF dof, double Nm);

    // Use this to scale down commanded forces (for your safety), range 0-1
    void setTorqueScale(double torqueScale){
        if(torqueScale>1 || torqueScale < 0) return;
        this->torqueScale = torqueScale;
    }


    void update();

    cVector3d getPosition() { return handlePos; }
    void setForce(cVector3d f);

protected:
    //double t0,t1,t2;
    bool enabled;
    S626DA **da;
    S626Encoder **encoder;
    double torqueScale;
    cVector3d handlePos;
    double motorAngle[3];
};

#endif // WOODENDEVICE_H
