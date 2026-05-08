#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"

/* SysTick 控制节拍相关变量。 */
extern unsigned int g_nMainEventCount;
extern volatile unsigned char g_u8ControlTickEnabled;

/* 姿态解算相关变量：角度环主要看 g_fCarAngle 和 g_fGyroAngleSpeed。 */
extern float g_fCarAngleOffset;
extern float g_fAccAngle;
extern float g_fGyroAngleSpeed;
extern float g_fCarAngle;

/* 编码器反馈：每 5ms 更新一次左右轮脉冲。 */
extern int g_nLeftMotorPulse;
extern int g_nRightMotorPulse;
extern int g_iGravity_Offset;

/* 电机保护和最终输出状态。 */
extern unsigned char g_cMotorDisable;
extern float g_fAngleControlOut;
extern float g_fLeftMotorOut;
extern float g_fRightMotorOut;

/* 速度环状态：目标速度、实际速度、速度环输出。 */
extern float g_fCarSpeed;
extern float g_fCarSpeedPrev;
extern float g_fCarSpeedSet;
extern float g_fCarSpeedSetTarget;
extern float g_fSpeedControlOut;

/* 转向/避障状态：蓝牙、超声波、循迹最终都会落到这些目标上。 */
extern float g_fBluetoothDirection;
extern float g_fBluetoothDirectionTarget;
extern int g_iLeftTurnRoundCnt;
extern int g_iRightTurnRoundCnt;
extern unsigned int g_nSpeedControlCount;
extern int g_nSpeedControlPeriod;

/* 读取 MPU6050 原始数据。 */
void GetMpuData(void);
/* 根据 MPU6050 数据计算车身角度。 */
void AngleCalculate(void);
/* 读取左右编码器脉冲，并累计给速度环使用。 */
void GetMotorPulse(void);
/* 电机安全管理：提起保护、倾倒保护。 */
void MotorManage(void);
/* 25ms 速度 PI 外环。 */
void SpeedControl(void);
/* 把 25ms 速度环输出平滑插值到更短控制周期。 */
void SpeedControlOutput(void);
/* 底层电机方向和 PWM 输出。 */
void SetMotorVoltageAndDirection(int nLeftMotorPwm, int nRightMotorPwm);
/* 合成角度环、速度环、转向量，得到左右电机输出。 */
void MotorOutput(void);
/* 5ms 角度 PD 内环。 */
void AngleControl(void);
/* 停止/切模式时清除保护和速度环残留。 */
void MotorClearAbnormalSpin(void);
/* 运动命令到来时解除保护，但不清空斜坡。 */
void MotorResumeFromProtection(void);
/* 每 1ms 平滑逼近速度/转向目标。 */
void ControlRampUpdate(void);
/* 按键机械零点校准。 */
void ControlCalibrateZeroAngle(void);
/* 比例映射函数，把输入范围映射到输出范围。 */
float Scale(float input, float input_min, float input_max, float output_min, float output_max);
/* 上层运动统一入口：direct 控转向，speed 控前后。 */
void Steer(float direct, float speed);
/* 超声波跟随/避障控制。mode=0 跟随，非 0 避障。 */
void UltraControl(int mode);

#endif
