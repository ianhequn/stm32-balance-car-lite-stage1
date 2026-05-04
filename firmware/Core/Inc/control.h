#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"

extern unsigned int g_nMainEventCount;
extern unsigned int g_nGetPulseCount;
extern float g_fAccAngle;
extern float g_fGyroAngleSpeed;
extern float g_fCarAngle;
extern int g_nLeftMotorPulse;
extern int g_nRightMotorPulse;
extern int g_iGravity_Offset;
extern unsigned char g_cMotorDisable;
extern float g_fCarSpeed;
extern float g_fCarSpeedPrev;
extern float g_fCarSpeedSet;
extern float g_fSpeedControlOut;
extern float g_fAngleControlOut;
extern float g_fLeftMotorOut;
extern float g_fRightMotorOut;
extern float g_fBluetoothDirection;
extern int g_iLeftTurnRoundCnt;
extern int g_iRightTurnRoundCnt;
extern unsigned int g_nSpeedControlCount;
extern int g_nSpeedControlPeriod;

void GetMpuData(void);
void AngleCalculate(void);
void GetMotorPulse(void);
void MotorManage(void);
void SpeedControl(void);
void SpeedControlOutput(void);
void SetMotorVoltageAndDirection(int nLeftMotorPwm, int nRightMotorPwm);
void MotorOutput(void);
void AngleControl(void);
void MotorClearAbnormalSpin(void);
float Scale(float input, float input_min, float input_max, float output_min, float output_max);
void Steer(float direct, float speed);
void UltraControl(int mode);

#endif
