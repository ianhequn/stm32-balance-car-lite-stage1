#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "tim.h"
#include "main.h"
#include "ultrasonic.h"

#define MOTOR_OUT_DEAD_VAL       0
#define MOTOR_OUT_MAX           999
#define MOTOR_OUT_MIN         (-999)

#define CAR_ZERO_ANGLE          0.0f
#define CAR_ANGLE_SET           CAR_ZERO_ANGLE
#define CAR_ANGLE_SPEED_SET     0

#define CAR_POSITION_MAX        900
#define CAR_POSITION_MIN      (-900)
#define SPEED_CONTROL_PERIOD    25

/* Stage-1 demo tuning: slower and easier to film. */
#define MANUAL_SPEED_OUT_MAX     50.0f
#define MANUAL_TURN_OUT_MAX     300.0f

#define ULTRA_FOLLOW_TARGET_CM      16.0f
#define ULTRA_FOLLOW_DEAD_ZONE_CM    2.0f
#define ULTRA_FOLLOW_NEAR_CM         5.0f
#define ULTRA_FOLLOW_FAR_CM         45.0f
#define ULTRA_FOLLOW_SPEED_MAX       2.6f
#define ULTRA_FOLLOW_KP              0.26f
#define ULTRA_DISTANCE_FILTER_A      0.82f

short x_nAcc, y_nAcc, z_nAcc;
short x_nGyro, y_nGyro, z_nGyro;
float x_fAcc, y_fAcc, z_fAcc;

float g_fAccAngle;
float g_fGyroAngleSpeed;
float g_fCarAngle;
float dt = 0.005f;

unsigned int g_nMainEventCount;
unsigned int g_nGetPulseCount;

int g_nLeftMotorPulse, g_nRightMotorPulse;
long g_lLeftMotorPulseSigma, g_lRightMotorPulseSigma;

int nPwmBais;
int nLeftMotorPwm, nRightMotorPwm;
int nLeftMotorErrorPrev, nRightMotorErrorPrev;
int g_iGravity_Offset = 0;
unsigned char g_cMotorDisable = 0;

float g_fLeftMotorOut, g_fRightMotorOut;
float g_fAngleControlOut;

float g_fCarSpeed;
float g_fCarSpeedPrev;
float g_fCarSpeedSet;
float g_fCarPosition;
float g_fSpeedControlOut;
float g_fSpeedControlOutOld;
float g_fSpeedControlOutNew;
unsigned int g_nSpeedControlCount;
int g_nSpeedControlPeriod;
float g_fBluetoothDirection;
int g_iLeftTurnRoundCnt;
int g_iRightTurnRoundCnt;
static int AbnormalSpinFlag = 0;

float g_fAngle_P = 43.0f;
float g_fAngle_D = 1.9f;
float g_fSpeed_P = 0.58f;
float g_fSpeed_I = 0.015f;


void GetMpuData(void)
{
    MPU_Get_Accelerometer(&x_nAcc, &y_nAcc, &z_nAcc);
    MPU_Get_Gyroscope(&x_nGyro, &y_nGyro, &z_nGyro);
}

void AngleCalculate(void)
{
    x_fAcc = x_nAcc / 16384.0f;
    y_fAcc = y_nAcc / 16384.0f;
    z_fAcc = z_nAcc / 16384.0f;

    g_fAccAngle = atan2(y_fAcc, z_fAcc) * 180.0f / 3.14f;
    g_fAccAngle = g_fAccAngle - g_iGravity_Offset;
    g_fGyroAngleSpeed = x_nGyro / 16.4f;
    g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);
}

void GetMotorPulse(void)
{
    g_nRightMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim4));
    g_nRightMotorPulse = -g_nRightMotorPulse;
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    g_nLeftMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    g_lLeftMotorPulseSigma += g_nLeftMotorPulse;
    g_lRightMotorPulseSigma += g_nRightMotorPulse;

    g_iLeftTurnRoundCnt -= g_nLeftMotorPulse;
    g_iRightTurnRoundCnt -= g_nRightMotorPulse;
}

static void AbnormalSpinDetect(short leftSpeed, short rightSpeed)
{
    static unsigned short count = 0;

    if ((int)g_fCarSpeedSet == 0)
    {
        if ((((leftSpeed > 30) && (rightSpeed > 30))
             || ((leftSpeed < -30) && (rightSpeed < -30)))
            && (g_fCarAngle > -30.0f) && (g_fCarAngle < 30.0f))
        {
            count++;
            if (count > 50)
            {
                count = 0;
                AbnormalSpinFlag = 1;
            }
        }
        else
        {
            count = 0;
        }
    }
    else
    {
        count = 0;
    }
}

static void LandingDetect(void)
{
    static float lastCarAngle = 0.0f;
    static unsigned short count = 0;
    static unsigned short count1 = 0;

    if (AbnormalSpinFlag == 0) return;

    if ((g_fCarAngle > -5.0f) && (g_fCarAngle < 5.0f))
    {
        count1++;
        if (count1 >= 50)
        {
            count1 = 0;
            if (((g_fCarAngle - lastCarAngle) < 0.8f)
                && ((g_fCarAngle - lastCarAngle) > -0.8f))
            {
                count++;
                if (count >= 4)
                {
                    count = 0;
                    count1 = 0;
                    g_fCarPosition = 0.0f;
                    AbnormalSpinFlag = 0;
                }
            }
            else
            {
                count = 0;
            }
            lastCarAngle = g_fCarAngle;
        }
    }
    else
    {
        count = 0;
        count1 = 0;
    }
}

void MotorClearAbnormalSpin(void)
{
    AbnormalSpinFlag = 0;
    g_fCarSpeedSet = 0.0f;
    g_fCarPosition = 0.0f;
    g_fSpeedControlOut = 0.0f;
    g_fSpeedControlOutOld = 0.0f;
    g_fSpeedControlOutNew = 0.0f;
    g_fBluetoothDirection = 0.0f;
    g_cMotorDisable &= ~(0x01 << 1);
}

void MotorManage(void)
{
    AbnormalSpinDetect((short)g_nLeftMotorPulse, (short)g_nRightMotorPulse);
    LandingDetect();

    if (AbnormalSpinFlag)
    {
        g_cMotorDisable |= (0x01 << 1);
    }
    else
    {
        g_cMotorDisable &= ~(0x01 << 1);
    }

    if (g_fCarAngle > 30.0f || g_fCarAngle < -30.0f)
    {
        g_cMotorDisable |= (0x01 << 2);
    }
    else
    {
        g_cMotorDisable &= ~(0x01 << 2);
    }
}

void SpeedControl(void)
{
    float fP, fI;
    float fDelta;

    g_fCarSpeed = (g_lLeftMotorPulseSigma + g_lRightMotorPulseSigma) * 0.5f;
    g_lLeftMotorPulseSigma = 0;
    g_lRightMotorPulseSigma = 0;

    g_fCarSpeed = 0.7f * g_fCarSpeedPrev + 0.3f * g_fCarSpeed;
    g_fCarSpeedPrev = g_fCarSpeed;

    fDelta = g_fCarSpeedSet;
    fDelta -= g_fCarSpeed;

    fP = fDelta * g_fSpeed_P;
    fI = fDelta * g_fSpeed_I;

    if (g_fCarSpeedSet > -0.1f && g_fCarSpeedSet < 0.1f)
    {
        /* Keep standing mode quiet: bleed off old speed-loop integral. */
        g_fCarPosition *= 0.92f;
        if (g_fCarPosition > -0.5f && g_fCarPosition < 0.5f)
        {
            g_fCarPosition = 0.0f;
        }
    }
    else
    {
        g_fCarPosition += fI;
    }

    if ((int)g_fCarPosition > CAR_POSITION_MAX) g_fCarPosition = CAR_POSITION_MAX;
    if ((int)g_fCarPosition < CAR_POSITION_MIN) g_fCarPosition = CAR_POSITION_MIN;

    g_fSpeedControlOutOld = g_fSpeedControlOutNew;
    g_fSpeedControlOutNew = fP + g_fCarPosition;
}

void SpeedControlOutput(void)
{
    float fValue;

    fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
    g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD
                       + g_fSpeedControlOutOld;
}

void SetMotorVoltageAndDirection(int nLeftMotorPwm, int nRightMotorPwm)
{
    if (nRightMotorPwm < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        nRightMotorPwm = -nRightMotorPwm;
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }

    if (nLeftMotorPwm < 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        nLeftMotorPwm = -nLeftMotorPwm;
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }

    if (nRightMotorPwm > MOTOR_OUT_MAX) nRightMotorPwm = MOTOR_OUT_MAX;
    if (nLeftMotorPwm > MOTOR_OUT_MAX) nLeftMotorPwm = MOTOR_OUT_MAX;

    if (g_cMotorDisable)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nRightMotorPwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nLeftMotorPwm);
    }
}

void MotorOutput(void)
{
    g_fLeftMotorOut = g_fAngleControlOut - g_fSpeedControlOut - g_fBluetoothDirection;
    g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection;

    if ((int)g_fLeftMotorOut > 0)       g_fLeftMotorOut += MOTOR_OUT_DEAD_VAL;
    else if ((int)g_fLeftMotorOut < 0)  g_fLeftMotorOut -= MOTOR_OUT_DEAD_VAL;
    if ((int)g_fRightMotorOut > 0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
    else if ((int)g_fRightMotorOut < 0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

    if ((int)g_fLeftMotorOut > MOTOR_OUT_MAX)    g_fLeftMotorOut = MOTOR_OUT_MAX;
    if ((int)g_fLeftMotorOut < MOTOR_OUT_MIN)    g_fLeftMotorOut = MOTOR_OUT_MIN;
    if ((int)g_fRightMotorOut > MOTOR_OUT_MAX)   g_fRightMotorOut = MOTOR_OUT_MAX;
    if ((int)g_fRightMotorOut < MOTOR_OUT_MIN)   g_fRightMotorOut = MOTOR_OUT_MIN;

    SetMotorVoltageAndDirection((int)g_fLeftMotorOut, (int)g_fRightMotorOut);
}

void AngleControl(void)
{
    g_fAngleControlOut = (CAR_ANGLE_SET - g_fCarAngle) * g_fAngle_P
                       + (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * g_fAngle_D;
}

float Scale(float input, float input_min, float input_max, float output_min, float output_max)
{
    float output;

    if (input_min < input_max)
    {
        output = (input - input_min) / ((input_max - input_min) / (output_max - output_min));
    }
    else
    {
        output = (input_min - input) / ((input_min - input_max) / (output_max - output_min));
    }

    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;

    return output;
}

void Steer(float direct, float speed)
{
    if (direct > 0.0f)
    {
        g_fBluetoothDirection = Scale(direct, 0.0f, 10.0f, 0.0f, MANUAL_TURN_OUT_MAX);
    }
    else
    {
        g_fBluetoothDirection = -Scale(direct, 0.0f, -10.0f, 0.0f, MANUAL_TURN_OUT_MAX);
    }

    if (speed > 0.0f)
    {
        g_fCarSpeedSet = Scale(speed, 0.0f, 10.0f, 0.0f, MANUAL_SPEED_OUT_MAX);
    }
    else
    {
        g_fCarSpeedSet = -Scale(speed, 0.0f, -10.0f, 0.0f, MANUAL_SPEED_OUT_MAX);
    }
}

void UltraControl(int mode)
{
    static float fDistanceFilt = ULTRA_FOLLOW_TARGET_CM;
    float fDistanceError;
    float fSpeedCmd;

    if (!IsUltraOK())
    {
        Steer(0.0f, 0.0f);
        return;
    }

    if (Distance < ULTRA_FOLLOW_NEAR_CM || Distance > ULTRA_FOLLOW_FAR_CM)
    {
        Steer(0.0f, 0.0f);
        return;
    }

    fDistanceFilt = ULTRA_DISTANCE_FILTER_A * fDistanceFilt
                  + (1.0f - ULTRA_DISTANCE_FILTER_A) * Distance;

    if (mode == 0)
    {
        fDistanceError = fDistanceFilt - ULTRA_FOLLOW_TARGET_CM;

        if (fDistanceError > -ULTRA_FOLLOW_DEAD_ZONE_CM
            && fDistanceError < ULTRA_FOLLOW_DEAD_ZONE_CM)
        {
            fSpeedCmd = 0.0f;
        }
        else
        {
            fSpeedCmd = fDistanceError * ULTRA_FOLLOW_KP;
            if (fSpeedCmd > ULTRA_FOLLOW_SPEED_MAX) fSpeedCmd = ULTRA_FOLLOW_SPEED_MAX;
            if (fSpeedCmd < -ULTRA_FOLLOW_SPEED_MAX) fSpeedCmd = -ULTRA_FOLLOW_SPEED_MAX;
        }

        Steer(0.0f, fSpeedCmd);
    }
    else
    {
        if (Distance > 0.0f && Distance <= 15.0f)
        {
            g_iLeftTurnRoundCnt = 500;
            g_iRightTurnRoundCnt = -500;
        }

        if (g_iLeftTurnRoundCnt > 0 || g_iRightTurnRoundCnt < 0)
        {
            Steer(5.0f, 0.0f);
        }
        else
        {
            Steer(0.0f, 4.0f);
        }
    }
}
