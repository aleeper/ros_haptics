//---------------------------------------------------------------------------
#if defined(C_ENABLE_WOODEN_DEVICE_SUPPORT)
//---------------------------------------------------------------------------

#include "woodendevice.h"
#include "s626comm.h"
#include <chai3d.h>

WoodenDevice::WoodenDevice(S626DA **da, S626Encoder **encoder) :
    da(da), encoder(encoder), enabled(false), torqueScale(1)
    //t0(0),t1(0),t2(0)
{
    std::cout << "Silicon Haptics presents WoodenDevice!" << std::endl;

}

WoodenDevice::WoodenDevice()
{
    S626Comm* daq = new S626Comm();
    daq->init();
    while(!daq->isInitialized()){
        std::cout << "Waiting to init s626" << std::endl;
        sleep(1);
    }
    da = daq->da;
    encoder = daq->encoder;
    enabled = true;
    torqueScale = 1;
}

void WoodenDevice::setMotorTorque(DOF dof, double Nm)
{
    if(!enabled)
        return;

    // Current capped in apmlifier to 3A. This means +10V signal -> 3A (?)
    const double maxCurrent = 3;
    const double maxon148877toqueConstant = 60.3e-3; // Nm/A from datasheet
    const double analogMax = da[0]->getAnalogMax();

    const double maxTorque = maxCurrent * maxon148877toqueConstant;
    if(Nm>maxTorque)
        Nm = maxTorque;
    if(Nm<-maxTorque)
        Nm = -maxTorque;

    double signal = (Nm/maxTorque)*analogMax * torqueScale;

    //std::cout << "setmotortorque " << signal << std::endl;

    da[dof]->writeVolt(signal);
}

bool WoodenDevice::wouldSaturate(DOF dof, double Nm){
    // Current capped in apmlifier to 3A. This means +10V signal -> 3A (?)
    const double maxCurrent = 2.0;
    const double maxon148877toqueConstant = 60.3e-3; // Nm/A from datasheet

    const double maxTorque = maxCurrent * maxon148877toqueConstant;
    if(cAbs(Nm)<maxTorque)
        return false;
    return true;
}

void WoodenDevice::update()
{
    for(int i=0;i<3;i++){
        encoder[i]->update();
        motorAngle[i] = encoder[i]->getAngle();
    }


    double gearRatio[3];
    gearRatio[0] = turntableDiameter / capstanDiameter0;
    gearRatio[1] = arm1Diameter / capstanDiameter1;
    gearRatio[2] = -1*arm2Diameter / capstanDiameter2;

    double dofAngle[3];
    for(int i=0;i<3;i++)
        dofAngle[i] = motorAngle[i]/gearRatio[i];

    // Do forward kinematics (thetas -> xyz)
    double t0,t1,t2; //global
    double l1,l2;

    l1 = dArm1Arm2;
    l2 = dArm2Handle;
    t0 = dofAngle[0];
    t1 = dofAngle[1];
    t2 = dofAngle[2] - t1;

    double fk[4];
    double Ln = dTurntableArm1;
    double Lb = dArm1Arm2;
    double Lc = dArm2Handle;
    double tA = t0;
    double tB = t1;
    double tC = t2;
    fk[1] = cos(tA)*(Lb*sin(tB)+Lc*cos(tB+tC));
    fk[2] = sin(tA)*(Lb*sin(tB)+Lc*cos(tB+tC));
    fk[3] = Ln + Lb*cos(tB) - Lc*sin(tB+tC);


    handlePos.set(fk[1],fk[2],fk[3]);




    /*
      Jonas & Mike
    handlePos.x = cos(t0) * (l2* cos(t1+t2) + l1*cos(t1)) + 0;
    handlePos.y = sin(t0) * (l2* cos(t1+t2) + l1*cos(t1)) + 0;
    handlePos.z =            l2* sin(t1+t2) + l1*sin(t1)  + dTurntableArm1;



*/
    // Agus
    /*
    double t3;
    t1 =  dofAngle[0];
    t2 = -dofAngle[1];
    t3 = -dofAngle[2] -t2 -M_PI_2;
    handlePos.zero();
    handlePos.y = l1*sin(t1)*cos(t2) + l2*sin(t1*t3);
    handlePos.z = l2-l2*cos(t3) + l1*sin(t2);
    handlePos.x = -l1+l1*cos(t1)*cos(t2) + l2*cos(t1)*sin(t3);
*/

}

void WoodenDevice::setForce(cVector3d f)
{


    for(int i=0;i<3;i++){
        motorAngle[i] = encoder[i]->getAngle();
    }


    double gearRatio[3];
    gearRatio[0] = turntableDiameter / capstanDiameter0;
    gearRatio[1] = arm1Diameter / capstanDiameter1;
    gearRatio[2] = -1*arm2Diameter / capstanDiameter2;

    double dofAngle[3];
    for(int i=0;i<3;i++)
        dofAngle[i] = motorAngle[i]/gearRatio[i];

    // Do forward kinematics (thetas -> xyz)
    double t0,t1,t2; //global
    double l1,l2;

    l1 = dArm1Arm2;
    l2 = dArm2Handle;
    t0 = dofAngle[0];
    t1 = dofAngle[1];
    t2 = dofAngle[2] - t1;





    double fk[4];
    double Ln = dTurntableArm1;
    double Lb = dArm1Arm2;
    double Lc = dArm2Handle;
    double tA = t0;
    double tB = t1;
    double tC = t2;

    double jac123[4][4];
    jac123[1][1] = -sin(tA)*(Lb*sin(tB)+Lc*cos(tB+tC));
    jac123[1][2] = Lb*cos(tA)*cos(tB);
    jac123[1][3] = -Lc*cos(tA)*sin(tB+tC);
    jac123[2][1] = cos(tA)*(Lb*sin(tB)+Lc*cos(tB+tC));
    jac123[2][2] = Lb*sin(tA)*cos(tB);
    jac123[2][3] = -Lc*sin(tA)*sin(tB+tC);
    jac123[3][1] = 0;
    jac123[3][2] = -Lb*sin(tB);
    jac123[3][3] = -Lc*cos(tB+tC);

    // the magic!
    cMatrix3d J;
    J.set(jac123[1][1],jac123[1][2],jac123[1][3],
          jac123[2][1],jac123[2][2],jac123[2][3],
          jac123[3][1],jac123[3][2],jac123[3][3]);


/*
    cMatrix3d gearRatios;
    double gr[3][3] = {{  capstanDiameter0/turntableDiameter, 0, 0 },
                      {  0, capstanDiameter1/arm1Diameter , 0     },
                      {  0, 0, capstanDiameter2/arm2Diameter   }   };
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
            gearRatios.m[i][j] = gr[i][j];
    }
    J.mul(gearRatios);
*/
    cVector3d t=cTranspose(J)*f;






    // Gravety comp!
    double g=9.81;
    double Lb_cm = Lb/4;
    double Lc_cm = Lc/2.2;
    //double mA = 0.3;
    double mB = 0.30;
    double mC = 0.070;
    double grav_comp[4];
    grav_comp[1] = -0;
    grav_comp[2] = -g*(Lb*mC*sin(tB)+Lb_cm*mB*sin(tB)+Lc_cm*mC*cos(tB+tC));
    grav_comp[3] = -g*(Lc_cm*mC*cos(tB+tC));// -Lb*mA*sin(tB)-mB*(Lb-Lb_cm)*sin(tB));



    t = t + cVector3d(grav_comp[1],grav_comp[2],grav_comp[3]);



    // Scale down gear ratio
    double tx,ty,tz;
    tx = t.x();
    ty = t.y();
    tz = t.z();

    tx = -tx * capstanDiameter0/turntableDiameter;
    ty = -ty * capstanDiameter1/arm1Diameter;
    tz = tz * capstanDiameter2/arm2Diameter;
    t.set(tx,ty,tz);

    // cap
    double scale = 1.0;
    double saturates = false;
    saturates = wouldSaturate(DOF(0),t.x()) ||
                wouldSaturate(DOF(1),t.y()) ||
                wouldSaturate(DOF(2),t.z());
    while (saturates){
        //std::cout << "scale " << scale << std::endl;
        scale = scale*0.95;
        saturates = wouldSaturate(DOF(0),t.x()*scale) ||
                    wouldSaturate(DOF(1),t.y()*scale) ||
                    wouldSaturate(DOF(2),t.z()*scale);
    }


    //std::cout << t.x()*scale << " " << t.y()*scale << " " << t.z() << std::endl;



    setMotorTorque(DOF(0),t.x()*scale);
    setMotorTorque(DOF(1),t.y()*scale);
    setMotorTorque(DOF(2),t.z()*scale);






    /* old crap
    // Do kinematics
    cMatrix3d J;
    double l2cos = l2*cos(t1+t2) + l1*cos(t1);
    double l2sin = l2*sin(t1+t2) + l1*sin(t1);

    double jm[3][3] = {
        { -sin(t0)*l2cos,  -cos(t0)*l2sin,  -cos(t0)*l2*sin(t1+t2) },
        { cos(t0)*l2cos,   -sin(t0)*l2sin,  -sin(t0)*l2*sin(t1+t2) },
        {  0            ,   l2cos,           l2*cos(t1+t2)         }};
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
            J.m[i][j] = jm[i][j];
    }


    cMatrix3d gearRatios;
    double g[3][3] = {{  capstanDiameter0/turntableDiameter, 0, 0 },
                      {  0, capstanDiameter1/arm1Diameter , 0     },
                      {  0, 0, capstanDiameter2/arm2Diameter   }   };
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
            gearRatios.m[i][j] = g[i][j];
    }


    J.trans();
    //J.mul(gearRatios);
    cVector3d torque = gearRatios* J * f;

    setMotorTorque(DOF(0),-torque.x);
    setMotorTorque(DOF(1),-torque.y);
    setMotorTorque(DOF(2),-torque.z);
*/
    /*
    // --------------------- jonas
    for(int i=0;i<3;i++){
        motorAngle[i] = encoder[i]->getAngle();
    }


    double gearRatio[3];
    gearRatio[0] = turntableDiameter / capstanDiameter0;
    gearRatio[1] = arm1Diameter / capstanDiameter1;
    gearRatio[2] = -1*arm2Diameter / capstanDiameter2;

    double dofAngle[3];
    for(int i=0;i<3;i++)
        dofAngle[i] = motorAngle[i]/gearRatio[i];

    // Do forward kinematics (thetas -> xyz)
    double t0,t1,t2; //global
    double l1,l2;

    l1 = dArm1Arm2;
    l2 = dArm2Handle;
    t0 = dofAngle[0];
    t1 = -dofAngle[1];
    t2 = -dofAngle[2] - t1;

    double torque[3] = { 0,0,0 };
    cVector3d p = handlePos;

    // First motor
    cVector3d r,fPlane;
    r = cVector3d(p.x,p.y,0);
    fPlane = cVector3d(f.x,f.y,0);
    r.cross(fPlane);
    torque[0] = r.z;
    std::cout << "r0  xyz " << r.x << "," << r.y << "," << r.z << std::endl;


    // second motor
    cVector3d midJoint = cVector3d(cos(t0)*dArm1Arm2*cos(t1),
                                   sin(t0)*dArm1Arm2*cos(t1),
                                   sin(t1)*dArm1Arm2 + dTurntableArm1);
    r = p-midJoint;
    fPlane = cVector3d(f.x,f.y,0);
    r.cross(fPlane);
    torque[2] = r.x;


    // third motor
    //r = cVector3d(p.x,p.y,p.z);
    //fPlane = cVector3d(f.x,f.y,0);

    // Scale down acording to gear ratio
    torque[0] = torque[0] * capstanDiameter0/turntableDiameter;
    torque[1] = torque[1] * capstanDiameter1/arm1Diameter;
    torque[2] = torque[2] * capstanDiameter2/arm2Diameter;

    torqueScale = 0.5;
    for(int i=0;i<3;i++)
        setMotorTorque(DOF(i),-torque[i]);
*/
}

#endif
