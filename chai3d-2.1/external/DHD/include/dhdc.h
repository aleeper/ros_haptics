/****************************************************************************
 *
 *  DHD - Haptic SDK ver 3.3.1
 *  Copyright (C) 2001-2011
 *  Force Dimension, Switzerland
 *  All Rights Reserved.
 *
 *  contact: support@forcedimension.com
 *
 ****************************************************************************/


/* C header */

#ifndef __DHDC_H__
#define __DHDC_H__

#include <cstddef>

/****************************************************************************
 *  OS DEPENDENCIES
 ****************************************************************************/

#if defined(WIN32) || defined(WIN64)
#ifndef __SDK
#define __SDK __stdcall
#endif
#endif

#ifndef __SDK
#define __SDK
#endif

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************
 *  TYPES
 ****************************************************************************/

typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;
typedef unsigned long  ulong;


/****************************************************************************
 *  ERROR MANAGEMENT
 ****************************************************************************/

  /* error codes */
  enum dhd_errors {
    DHD_NO_ERROR,
    DHD_ERROR,
    DHD_ERROR_COM,
    DHD_ERROR_DHC_BUSY,
    DHD_ERROR_NO_DRIVER_FOUND,
    DHD_ERROR_NO_DEVICE_FOUND,
    DHD_ERROR_NOT_AVAILABLE,
    DHD_ERROR_TIMEOUT,
    DHD_ERROR_GEOMETRY,
    DHD_ERROR_EXPERT_MODE_DISABLED,
    DHD_ERROR_NOT_IMPLEMENTED,
    DHD_ERROR_OUT_OF_MEMORY,
    DHD_ERROR_DEVICE_NOT_READY,
    DHD_ERROR_FILE_NOT_FOUND,
    DHD_ERROR_CONFIGURATION,
    DHD_ERROR_INVALID_INDEX,
    DHD_ERROR_DEPRECATED
  };

  /* error reporting */
  int         __SDK dhdErrorGetLast    ();
  const char* __SDK dhdErrorGetLastStr ();
  const char* __SDK dhdErrorGetStr     (int error);


/****************************************************************************
 *  CONSTANTS
 ****************************************************************************/

/* devices */
#define DHD_DEVICE_NONE            0
#define DHD_DEVICE_3DOF           31
#define DHD_DEVICE_6DOF           61
#define DHD_DEVICE_6DOF_500       62
#define DHD_DEVICE_3DOF_USB       63
#define DHD_DEVICE_6DOF_USB       64
#define DHD_DEVICE_OMEGA          32
#define DHD_DEVICE_OMEGA3         33
#define DHD_DEVICE_OMEGA33        34
#define DHD_DEVICE_OMEGA33_LEFT   36
#define DHD_DEVICE_OMEGA331       35
#define DHD_DEVICE_OMEGA331_LEFT  37
#define DHD_DEVICE_FALCON         60
#define DHD_DEVICE_CONTROLLER     81
#define DHD_DEVICE_CONTROLLER_HR  82
#define DHD_DEVICE_CUSTOM         91
#define DHD_DEVICE_SIGMA331      102
#define DHD_DEVICE_SIGMA331_LEFT 103

/* status */
#define DHD_ON                     1
#define DHD_OFF                    0

/* device count */
#define DHD_MAX_DEVICE             4

/* encoder count */
#define DHD_MAX_DOF                8

/* delta motor index */
#define DHD_DELTA_MOTOR_0          0
#define DHD_DELTA_MOTOR_1          1
#define DHD_DELTA_MOTOR_2          2

/* delta encoder index */
#define DHD_DELTA_ENC_0            0
#define DHD_DELTA_ENC_1            1
#define DHD_DELTA_ENC_2            2

/* wrist motor index */
#define DHD_WRIST_MOTOR_0          3
#define DHD_WRIST_MOTOR_1          4
#define DHD_WRIST_MOTOR_2          5

/* wrist encoder index */
#define DHD_WRIST_ENC_0            3
#define DHD_WRIST_ENC_1            4
#define DHD_WRIST_ENC_2            5

/* gripper encoder index */
#define DHD_GRIP_ENC               6
#define DHD_GRIP_MOT               3

/* useful non-error, positive return values */
#define DHD_TIMEGUARD              1
#define DHD_MOTOR_SATURATED        2

/* status count */
#define DHD_MAX_STATUS            13

/* status codes */
#define DHD_STATUS_POWER           0
#define DHD_STATUS_CONNECTED       1
#define DHD_STATUS_STARTED         2
#define DHD_STATUS_RESET           3
#define DHD_STATUS_IDLE            4
#define DHD_STATUS_FORCE           5
#define DHD_STATUS_BRAKE           6
#define DHD_STATUS_TORQUE          7
#define DHD_STATUS_WRIST_DETECTED  8
#define DHD_STATUS_ERROR           9
#define DHD_STATUS_GRAVITY        10
#define DHD_STATUS_TIMEGUARD      11
#define DHD_STATUS_WRIST_RESET    12

/* buttons count */
#define DHD_MAX_BUTTONS            8

/* velocity estimator computation mode */
#define DHD_VELOCITY_WINDOWING     0
#define DHD_VELOCITY_AVERAGING     1
#define DHD_VELOCITY_WINDOW       20  // [ms]


/****************************************************************************
 *  standard SDK
 ****************************************************************************/

  int         __SDK dhdGetDeviceCount                    ();
  int         __SDK dhdSetDevice                         (char ID);
  int         __SDK dhdGetDeviceID                       ();
  int         __SDK dhdGetSerialNumber                   (ushort *sn, char ID = -1);
  int         __SDK dhdOpen                              ();
  int         __SDK dhdOpenID                            (char ID);
  int         __SDK dhdClose                             (char ID = -1);
  int         __SDK dhdStop                              (char ID = -1);
  int         __SDK dhdGetSystemType                     (char ID = -1);
  const char* __SDK dhdGetSystemName                     (char ID = -1);                                                                                           /* added in 3.2 release */
  int         __SDK dhdGetVersion                        (double *ver, char ID = -1);
  void        __SDK dhdGetSDKVersion                     (int *major, int *minor, int *release, int *revision);
  int         __SDK dhdGetStatus                         (int status[DHD_MAX_STATUS], char ID = -1);
  int         __SDK dhdGetDeviceAngleRad                 (double *angle, char ID = -1);
  int         __SDK dhdGetDeviceAngleDeg                 (double *angle, char ID = -1);
  int         __SDK dhdGetEffectorMass                   (double *mass, char ID = -1);
  ulong       __SDK dhdGetSystemCounter                  ();
  int         __SDK dhdGetButton                         (int index, char ID = -1);
  bool        __SDK dhdIsLeftHanded                      (char ID = -1);
  int         __SDK dhdReset                             (char ID = -1);
  int         __SDK dhdResetWrist                        (char ID = -1);
  int         __SDK dhdWaitForReset                      (int timeout = 0, char ID = -1);
  int         __SDK dhdSetStandardGravity                (double g, char ID = -1);
  int         __SDK dhdSetGravityCompensation            (int val = DHD_ON, char ID = -1);
  int         __SDK dhdSetBrakes                         (int val = DHD_ON, char ID = -1);
  int         __SDK dhdSetDeviceAngleRad                 (double angle, char ID = -1);
  int         __SDK dhdSetDeviceAngleDeg                 (double angle, char ID = -1);
  int         __SDK dhdSetEffectorMass                   (double mass,  char ID = -1);
  int         __SDK dhdGetPosition                       (double *px, double *py, double *pz, char ID = -1);
  int         __SDK dhdGetForce                          (double *fx, double *fy, double *fz, char ID = -1);
  int         __SDK dhdSetForce                          (double  fx, double  fy, double  fz, char ID = -1);
  int         __SDK dhdGetOrientationRad                 (double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdGetOrientationDeg                 (double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdGetTorque                         (double *ta, double *tb, double *tg, char ID = -1);
  int         __SDK dhdSetTorque                         (double  ta, double  tb, double  tg, char ID = -1);
  int         __SDK dhdGetPositionAndOrientationRad      (double *px, double *py, double *pz, double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdGetPositionAndOrientationDeg      (double *px, double *py, double *pz, double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdGetPositionAndOrientationFrame    (double *px, double *py, double *pz, double matrix[3][3], char ID = -1);
  int         __SDK dhdGetForceAndTorque                 (double *fx, double *fy, double *fz, double *ta, double *tb, double *tg, char ID = -1);
  int         __SDK dhdSetForceAndTorque                 (double  fx, double  fy, double  fz, double  ta, double  tb, double  tg, char ID = -1);
  int         __SDK dhdGetOrientationFrame               (double matrix[3][3], char ID = -1);
  int         __SDK dhdGetGripperAngleDeg                (double *a, char ID = -1);
  int         __SDK dhdGetGripperAngleRad                (double *a, char ID = -1);
  int         __SDK dhdGetGripperThumbPos                (double *px, double *py, double *pz,  char ID = -1);
  int         __SDK dhdGetGripperFingerPos               (double *px, double *py, double *pz,  char ID = -1);
  int         __SDK dhdSetGripperTorque                  (double t, char ID = -1);
  int         __SDK dhdSetGripperForce                   (double f, char ID = -1);
  double      __SDK dhdGetComFreq                        (char ID = -1);
  int         __SDK dhdSetForceAndGripperForce           (double fx, double fy, double fz, double f, char ID = -1);
  int         __SDK dhdSetForceAndGripperTorque          (double fx, double fy, double fz, double t, char ID = -1);
  int         __SDK dhdSetForceAndTorqueAndGripperForce  (double fx, double fy, double fz, double ta, double tb, double tg, double f, char ID = -1);               /* added in 3.3 release */
  int         __SDK dhdSetForceAndTorqueAndGripperTorque (double fx, double fy, double fz, double ta, double tb, double tg, double t, char ID = -1);               /* added in 3.3 release */
  int         __SDK dhdGetForceAndTorqueAndGripperForce  (double *fx, double *fy, double *fz, double *ta, double *tb, double *tg, double *f, char ID = -1);        /* added in 3.3 release */
  int         __SDK dhdGetForceAndTorqueAndGripperTorque (double *fx, double *fy, double *fz, double *ta, double *tb, double *tg, double *t, char ID = -1);        /* added in 3.3 release */
  int         __SDK dhdConfigLinearVelocity              (int ms = DHD_VELOCITY_WINDOW, int mode = DHD_VELOCITY_WINDOWING, char ID = -1);
  int         __SDK dhdGetLinearVelocity                 (double *vx, double *vy, double *vz, char ID = -1);
  int         __SDK dhdEmulateButton                     (uchar val, char ID = -1);
  int         __SDK dhdGetBaseAngleXRad                  (double *angle, char ID = -1);                                                                            /* added in 3.3 release */
  int         __SDK dhdGetBaseAngleXDeg                  (double *angle, char ID = -1);                                                                            /* added in 3.3 release */
  int         __SDK dhdSetBaseAngleXRad                  (double angle, char ID = -1);                                                                             /* added in 3.3.1 release */
  int         __SDK dhdSetBaseAngleXDeg                  (double angle, char ID = -1);                                                                             /* added in 3.3.1 release */
  int         __SDK dhdGetBaseAngleZRad                  (double *angle, char ID = -1);                                                                            /* added in 3.3 release */
  int         __SDK dhdGetBaseAngleZDeg                  (double *angle, char ID = -1);                                                                            /* added in 3.3 release */
  int         __SDK dhdSetBaseAngleZRad                  (double angle, char ID = -1);                                                                             /* added in 3.3 release */
  int         __SDK dhdSetBaseAngleZDeg                  (double angle, char ID = -1);                                                                             /* added in 3.3 release */


/****************************************************************************
 *  expert SDK
 ****************************************************************************/

  int         __SDK dhdEnableExpertMode                  ();
  int         __SDK dhdDisableExpertMode                 ();
  int         __SDK dhdPreset                            (int val[DHD_MAX_DOF], uchar mask, char ID = -1);
  int         __SDK dhdEnableForce                       (uchar val, char ID = -1);
  int         __SDK dhdCalibrateWrist                    (char ID = -1);
  int         __SDK dhdSetTimeGuard                      (int us,  char ID = -1);
  int         __SDK dhdSetVelocityThreshold              (uchar val, char ID = -1);
  int         __SDK dhdGetVelocityThreshold              (uchar *val, char ID = -1);
  int         __SDK dhdUpdateEncoders                    (char ID = -1);
  int         __SDK dhdGetDeltaEncoders                  (int *enc0, int *enc1, int *enc2, char ID = -1);
  int         __SDK dhdGetWristEncoders                  (int *enc0, int *enc1, int *enc2, char ID = -1);
  int         __SDK dhdGetGripperEncoder                 (int *enc, char ID = -1);
  int         __SDK dhdGetEncoder                        (int index, char ID = -1);
  int         __SDK dhdSetMotor                          (int index, short val, char ID = -1);
  int         __SDK dhdSetDeltaMotor                     (short mot0, short mot1, short mot2, char ID = -1);
  int         __SDK dhdSetWristMotor                     (short mot0, short mot1, short mot2, char ID = -1);
  int         __SDK dhdSetGripperMotor                   (short mot, char ID = -1);
  int         __SDK dhdDeltaEncoderToPosition            (int  enc0, int  enc1, int  enc2, double *px, double *py, double *pz, char ID = -1);
  int         __SDK dhdDeltaPositionToEncoder            (double px, double py, double pz, int  *enc0, int  *enc1, int  *enc2, char ID = -1);
  int         __SDK dhdDeltaMotorToForce                 (short mot0, short mot1, short mot2, int enc0, int enc1, int enc2, double  *fx, double  *fy, double  *fz, char ID = -1);
  int         __SDK dhdDeltaForceToMotor                 (double  fx, double  fy, double  fz, int enc0, int enc1, int enc2, short *mot0, short *mot1, short *mot2, char ID = -1);
  int         __SDK dhdWristEncoderToOrientation         (int  enc0, int  enc1, int  enc2, double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdWristOrientationToEncoder         (double oa, double ob, double og, int  *enc0, int  *enc1, int  *enc2, char ID = -1);
  int         __SDK dhdWristMotorToTorque                (short mot0, short mot1, short mot2, int enc0, int enc1, int enc2, double  *ta, double  *tb, double  *tg, char ID = -1);
  int         __SDK dhdWristTorqueToMotor                (double  ta, double  tb, double  tg, int enc0, int enc1, int enc2, short *mot0, short *mot1, short *mot2, char ID = -1);
  int         __SDK dhdGripperEncoderToOrientation       (int enc, double *a, char ID = -1);
  int         __SDK dhdGripperEncoderToPosition          (int enc, double *p, char ID = -1);
  int         __SDK dhdGripperOrientationToEncoder       (double a, int *enc, char ID = -1);
  int         __SDK dhdGripperPositionToEncoder          (double p, int *enc, char ID = -1);
  int         __SDK dhdGripperMotorToTorque              (short mot, double *t, int e = 0, double rWo[3][3] = NULL, char ID = -1);
  int         __SDK dhdGripperMotorToForce               (short mot, double *f, int e = 0, double rWo[3][3] = NULL, char ID = -1);
  int         __SDK dhdGripperTorqueToMotor              (double t, short *mot, int e = 0, double rWo[3][3] = NULL, char ID = -1);
  int         __SDK dhdGripperForceToMotor               (double f, short *mot, int e = 0, double rWo[3][3] = NULL, char ID = -1);
  int         __SDK dhdSetMot                            (short mot[DHD_MAX_DOF], uchar mask = 0xff, char ID = -1);
  int         __SDK dhdGetEnc                            (int   enc[DHD_MAX_DOF], uchar mask = 0xff, char ID = -1);
  int         __SDK dhdSetBrk                            (uchar mask = 0xff, char ID = -1);
  int         __SDK dhdGetDeltaJointAngles               (double *j0, double *j1, double *j2, char ID = -1);                                                       /* added in 3.3 release */
  int         __SDK dhdGetDeltaJacobian                  (double jcb[3][3], char ID = -1);                                                                         /* added in 3.3 release */
  int         __SDK dhdDeltaJointAnglesToJacobian        (double j0, double j1, double j2, double jcb[3][3], char ID = -1);                                        /* added in 3.3 release */
  int         __SDK dhdDeltaJointTorquesExtrema          (double j0, double j1, double j2, double minq[3], double maxq[3], char ID = -1);                          /* added in 3.3 release */
  int         __SDK dhdDeltaGravityJointTorques          (double j0, double j1, double j2, double *q0, double *q1, double *q2, char ID = -1);                      /* added in 3.3 release */
  int         __SDK dhdSetDeltaJointTorques              (double t0, double t1, double t2, char ID = -1);                                                          /* added in 3.3 release */
  int         __SDK dhdSetAsyncMode                      (bool async, char ID = -1);                                                                               /* added in 3.3 release */
  bool        __SDK dhdGetAsyncMode                      (char ID = -1);                                                                                           /* added in 3.3 release */


/****************************************************************************
 *  controller SDK
 ****************************************************************************/

  int         __SDK dhdControllerSetDevice               (int device, char ID = -1);
  int         __SDK dhdReadConfigFromFile                (char *filename, char ID = -1);


/****************************************************************************
 *  OS independent utilities
 ****************************************************************************/

  bool        __SDK dhdKbHit                             ();
  char        __SDK dhdKbGet                             ();
  double      __SDK dhdGetTime                           ();
  void        __SDK dhdSleep                             (double sec);


#ifdef __cplusplus
}
#endif


#endif
