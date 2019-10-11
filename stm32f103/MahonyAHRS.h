#ifndef _MahonyAHRS_
#define _MahonyAHRS_

#include "IMU.h"
#include <arm_math.h>

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(IMU_Accel acc, IMU_Gyro gyro, IMU_Mag mag, IMU_Angle* angle, bool updated);
void MahonyAHRSupdateIMU(IMU_Accel acc, IMU_Gyro gyro, IMU_Angle* angle);

#endif //_MahonyAHRS_
