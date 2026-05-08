#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "tim.h"
#include "main.h"
#include "ultrasonic.h"

/*
 * control.c 是整车控制核心：
 * MPU6050 和编码器提供反馈，角度环负责站稳，速度环负责前后速度，
 * Steer() 接收蓝牙/超声波/循迹的运动意图，MotorOutput() 合成最终 PWM。
 */

/* 电机低 PWM 常常推不动齿轮箱，所以需要死区补偿。 */
#define MOTOR_OUT_DEAD_VAL       31.0f
/* 小于这个范围的输出当作噪声，不给电机，避免原地轻微抖动。 */
#define MOTOR_OUT_NOISE_BAND      8.0f
#define MOTOR_OUT_MAX           999
#define MOTOR_OUT_MIN         (-999)

/* 机械零点试验版：先固定为 0，验证原地抖动是否来自零点偏移。 */
#define CAR_ZERO_ANGLE          0.0f
#define CAR_ANGLE_SET           CAR_ZERO_ANGLE
#define CAR_ANGLE_SPEED_SET     0

/* 速度环积分限幅。太大时会出现“先推不动，动起来后突然冲”。 */
#define CAR_POSITION_MAX        550
#define CAR_POSITION_MIN      (-550)
#define SPEED_CONTROL_PERIOD    25

/* 手动模式最大速度/转向输出，先保守一点，方便测试和录视频。 */
#define MANUAL_SPEED_OUT_MAX     50.0f
#define MANUAL_TURN_OUT_MAX     300.0f
/* 每 1ms 目标速度最多变化多少。越大响应越快，越小越柔。 */
#define SPEED_RAMP_STEP           0.35f
/* 每 1ms 转向差速最多变化多少。用于让 L/R 不再硬跳变。 */
#define DIRECTION_RAMP_STEP       5.0f
/* 速度环积分每 25ms 最大增加量，防止低速卡住时积分越积越多。 */
#define SPEED_INTEGRAL_STEP_MAX   0.45f
/* 静止目标死区：目标速度非常接近 0 时，认为当前就是停止状态。 */
#define SPEED_STOP_SET_DEAD_BAND       0.12f
/* 静止反馈死区：停止状态下编码器只有很小脉冲时，按速度为 0 处理。 */
#define SPEED_STOP_FEEDBACK_DEAD_BAND  1.20f
/* 按键校准机械零点时，只允许在车身接近直立时校准。 */
#define ZERO_CALIBRATE_LIMIT      8.0f

/* 超声波跟随参数：目标距离、死区、有效距离范围、速度限幅和滤波。 */
#define ULTRA_FOLLOW_TARGET_CM      16.0f
#define ULTRA_FOLLOW_DEAD_ZONE_CM    2.0f
#define ULTRA_FOLLOW_NEAR_CM         5.0f
#define ULTRA_FOLLOW_FAR_CM         45.0f
#define ULTRA_FOLLOW_SPEED_MAX       2.6f
#define ULTRA_FOLLOW_KP              0.26f
#define ULTRA_DISTANCE_FILTER_A      0.82f

/* MPU6050 原始数据：加速度和陀螺仪都是 16 位有符号数。 */
short x_nAcc, y_nAcc, z_nAcc;
short x_nGyro, y_nGyro, z_nGyro;
/* 加速度换算成 g 之后的浮点值，后面用来算倾角。 */
float x_fAcc, y_fAcc, z_fAcc;

/* 加速度算出来的角度，短时不准但长期不漂。 */
float g_fAccAngle;
/* 陀螺仪角速度，短时灵敏但长期会漂。 */
float g_fGyroAngleSpeed;
/* 互补滤波后的车身角度，是角度环真正使用的反馈量。 */
float g_fCarAngle;
/* 机械零点偏移值，按板子按键可以把当前角度校准进来。 */
float g_fCarAngleOffset = 0.0f;
/* 角度解算周期，当前 5ms 算一次，所以 dt = 0.005s。 */
float dt = 0.005f;

/* SysTick 里的主控制分片计数，用 1/2/3/4 片错开任务，降低瞬时负载。 */
unsigned int g_nMainEventCount;
/* 控制节拍开关：MPU 初始化成功后再允许控制中断真正工作。 */
volatile unsigned char g_u8ControlTickEnabled;

/* 每 5ms 从 TIM2/TIM4 读到的左右轮编码器增量。 */
int g_nLeftMotorPulse, g_nRightMotorPulse;
/* 速度环用的 25ms 编码器累计量：5 次 5ms 脉冲相加后给 SpeedControl()。 */
long g_lLeftMotorPulseSigma, g_lRightMotorPulseSigma;

/* 角度安装/重力方向修正常量，目前保留为 0。 */
int g_iGravity_Offset = 0;
/* 电机禁用位图：任意一位为 1 时，SetMotorVoltageAndDirection() 会关 PWM。 */
unsigned char g_cMotorDisable = 0;

/* 最终给左右电机的带符号 PWM，正负代表方向，绝对值代表占空比。 */
float g_fLeftMotorOut, g_fRightMotorOut;
/* 角度环输出：主要负责让车身不倒。 */
float g_fAngleControlOut;

/* 当前速度反馈，单位是 25ms 内左右轮平均脉冲数。 */
float g_fCarSpeed;
/* 上一次速度反馈，用于 0.7/0.3 低通滤波。 */
float g_fCarSpeedPrev;
/* 实际进入速度环的目标速度，由 ControlRampUpdate() 平滑逼近目标值。 */
float g_fCarSpeedSet;
/* 蓝牙/超声波/循迹设置的是目标值，不直接阶跃进入速度环。 */
float g_fCarSpeedSetTarget;
/* 速度 PI 的积分量，也可以理解为“位置/路程累计误差”。 */
float g_fCarPosition;
/* 当前速度环输出，最终会从角度环输出里减掉。 */
float g_fSpeedControlOut;
/* 速度环插值的旧值和新值，用来把 25ms 更新平滑展开。 */
float g_fSpeedControlOutOld;
float g_fSpeedControlOutNew;
/* 速度环 25ms 分频计数。 */
unsigned int g_nSpeedControlCount;
/* 速度环输出插值周期计数，每 1ms 增加，用于 SpeedControlOutput()。 */
int g_nSpeedControlPeriod;
/* 实际进入 MotorOutput() 的转向差速，由 ControlRampUpdate() 平滑逼近。 */
float g_fBluetoothDirection;
/* 蓝牙/超声波/循迹设置的转向目标。 */
float g_fBluetoothDirectionTarget;
/* 超声波避障转向计数：靠编码器脉冲递减，控制转弯持续距离。 */
int g_iLeftTurnRoundCnt;
int g_iRightTurnRoundCnt;
/* 提起/空转保护标志：检测到轮子空转后先停电机，落地稳定后恢复。 */
static int AbnormalSpinFlag = 0;

/* 角度环 PD 参数：P 抗倾倒，D 抑制角速度。 */
float g_fAngle_P = 38.0f;
float g_fAngle_D = 1.6f;
/* 速度环 PI 参数：P 响应速度误差，I 抑制长期漂移。 */
float g_fSpeed_P = 0.18f;
float g_fSpeed_I = 0.003f;
/* 通用限幅函数：只限制数值范围，不做单位映射。 */
static float LimitFloat(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/* 斜坡逼近函数：让 current 每次最多变化 step，避免目标值阶跃。 */
static float ApproachFloat(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target) current = target;
    }
    else if (current > target)
    {
        current -= step;
        if (current < target) current = target;
    }

    return current;
}

/* 电机死区补偿：小输出归零，有效输出额外加一段克服静摩擦。 */
static float ApplyMotorDeadZone(float value)
{
    /* 电机低 PWM 会被静摩擦吃掉；小噪声归零，真正动作再补偿死区。 */
    if (value > -MOTOR_OUT_NOISE_BAND && value < MOTOR_OUT_NOISE_BAND)
    {
        return 0.0f;
    }

    if (value > 0.0f) return value + MOTOR_OUT_DEAD_VAL;
    return value - MOTOR_OUT_DEAD_VAL;
}


void GetMpuData(void)
{
    /* 从 MPU6050 读取一次原始加速度和陀螺仪数据。 */
    MPU_Get_Accelerometer(&x_nAcc, &y_nAcc, &z_nAcc);
    MPU_Get_Gyroscope(&x_nGyro, &y_nGyro, &z_nGyro);
}

void AngleCalculate(void)
{
    /* 加速度量程按 +-2g 配置时，16384 LSB 约等于 1g。 */
    x_fAcc = x_nAcc / 16384.0f;
    y_fAcc = y_nAcc / 16384.0f;
    z_fAcc = z_nAcc / 16384.0f;

    /* 用 y/z 加速度反推车身倾角，单位转成度。 */
    g_fAccAngle = atan2(y_fAcc, z_fAcc) * 180.0f / 3.14f;
    g_fAccAngle = g_fAccAngle - g_iGravity_Offset;
    /* 陀螺仪量程按 +-2000dps 时，16.4 LSB 约等于 1 度/秒。 */
    g_fGyroAngleSpeed = x_nGyro / 16.4f;
    /* 互补滤波：加速度修正长期角度，陀螺仪保证短时响应。 */
    g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);
    /* 机械零点补偿：修正模块/电池安装导致的前后重心偏移。 */
    g_fCarAngle = g_fCarAngle - CAR_ZERO_ANGLE;
}

void GetMotorPulse(void)
{
    /* 右轮编码器在本车安装方向下需要取反，保证前进时左右轮速度符号一致。 */
    g_nRightMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim4));
    g_nRightMotorPulse = -g_nRightMotorPulse;
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    /* 左轮编码器直接读取 TIM2 计数值，然后清零，等待下一个 5ms 周期。 */
    g_nLeftMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* 速度环 25ms 执行一次，所以每 5ms 先累计，满 25ms 再统一计算速度。 */
    g_lLeftMotorPulseSigma += g_nLeftMotorPulse;
    g_lRightMotorPulseSigma += g_nRightMotorPulse;

    /* 避障转弯用编码器脉冲当“转了多远”的计数依据。 */
    g_iLeftTurnRoundCnt -= g_nLeftMotorPulse;
    g_iRightTurnRoundCnt -= g_nRightMotorPulse;
}

/* 提起/空转检测：目标速度为 0 但两个轮子持续同向高速转，认为车被提起或异常空转。 */
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

/* 落地检测：空转保护后，角度回到接近直立且变化很小，才解除保护。 */
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
    /* 完整停止/切模式时调用：清保护、清速度目标、清速度环残留。 */
    AbnormalSpinFlag = 0;
    g_fCarSpeedSetTarget = 0.0f;
    g_fCarSpeedSet = 0.0f;
    g_fCarPosition = 0.0f;
    g_fSpeedControlOut = 0.0f;
    g_fSpeedControlOutOld = 0.0f;
    g_fSpeedControlOutNew = 0.0f;
    g_fBluetoothDirectionTarget = 0.0f;
    g_fBluetoothDirection = 0.0f;
    g_cMotorDisable &= ~(0x01 << 1);
}

void MotorResumeFromProtection(void)
{
    /* 运动命令只解除提起保护，不清空当前速度斜坡，避免按键连发造成顿挫。 */
    /* 遥控运动命令只解除提起保护，不清空当前速度斜坡，避免按键连发造成顿挫。 */
    AbnormalSpinFlag = 0;
    g_cMotorDisable &= ~(0x01 << 1);
}

void ControlRampUpdate(void)
{
    /* 蓝牙/超声波/循迹给的是目标值；这里每 1ms 平滑逼近，避免阶跃冲车。 */
    /* 蓝牙/超声波/循迹给的是目标值；这里每 1ms 平滑逼近，避免阶跃冲车。 */
    g_fCarSpeedSet = ApproachFloat(g_fCarSpeedSet,
                                   g_fCarSpeedSetTarget,
                                   SPEED_RAMP_STEP);
    g_fBluetoothDirection = ApproachFloat(g_fBluetoothDirection,
                                          g_fBluetoothDirectionTarget,
                                          DIRECTION_RAMP_STEP);
}

void ControlCalibrateZeroAngle(void)
{
    /* 当前试验固定机械零点为 0：按键只清空偏移值和速度环残留。 */
    if (g_fCarAngle > -ZERO_CALIBRATE_LIMIT && g_fCarAngle < ZERO_CALIBRATE_LIMIT)
    {
        g_fCarAngleOffset = 0.0f;
        MotorClearAbnormalSpin();
    }
}

void MotorManage(void)
{
    /*
     * 电机安全管理：
     * 1. 判断是否提起/空转；
     * 2. 判断车身是否倾倒超过安全角度；
     * 3. 通过 g_cMotorDisable 的不同 bit 控制电机是否允许输出。
     */
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

    /*
     * 速度环 25ms 执行一次：
     * 先把 5ms 累计的左右编码器脉冲求平均，再低通滤波。
     * 误差 = 目标速度 - 当前速度，P 项负责快速响应，I 项负责消除长期漂移。
     */
    g_fCarSpeed = (g_lLeftMotorPulseSigma + g_lRightMotorPulseSigma) * 0.5f;
    g_lLeftMotorPulseSigma = 0;
    g_lRightMotorPulseSigma = 0;

    /* 速度反馈稍微滤得更重一点，减少原地编码器小脉冲带来的速度环抖动。 */
    g_fCarSpeed = 0.8f * g_fCarSpeedPrev + 0.2f * g_fCarSpeed;
    g_fCarSpeedPrev = g_fCarSpeed;

    fDelta = g_fCarSpeedSet;
    fDelta -= g_fCarSpeed;

    if (g_fCarSpeedSet > -SPEED_STOP_SET_DEAD_BAND
        && g_fCarSpeedSet < SPEED_STOP_SET_DEAD_BAND
        && g_fCarSpeed > -SPEED_STOP_FEEDBACK_DEAD_BAND
        && g_fCarSpeed < SPEED_STOP_FEEDBACK_DEAD_BAND)
    {
        /*
         * 静止速度死区：
         * S 停止后，如果目标速度接近 0 且编码器反馈只是很小抖动，
         * 就不要让速度环继续放大小误差，减少原地来回抽动。
         */
        fDelta = 0.0f;
        g_fCarSpeed = 0.0f;
        g_fCarSpeedPrev = 0.0f;
    }

    fP = fDelta * g_fSpeed_P;
    fI = fDelta * g_fSpeed_I;
    /* 单次积分限幅：低速卡住时不要让积分一直堆，避免起步后突然释放。 */
    fI = LimitFloat(fI, -SPEED_INTEGRAL_STEP_MAX, SPEED_INTEGRAL_STEP_MAX);

    if (g_fCarSpeedSet > -SPEED_STOP_SET_DEAD_BAND
        && g_fCarSpeedSet < SPEED_STOP_SET_DEAD_BAND)
    {
        /* 停止时加快释放速度环积分，提升 S/刹停响应。 */
        g_fCarPosition *= 0.75f;
        if (g_fCarPosition > -0.5f && g_fCarPosition < 0.5f)
        {
            g_fCarPosition = 0.0f;
        }
    }
    else
    {
        g_fCarPosition += fI;
    }

    g_fCarPosition = LimitFloat(g_fCarPosition, CAR_POSITION_MIN, CAR_POSITION_MAX);

    g_fSpeedControlOutOld = g_fSpeedControlOutNew;
    g_fSpeedControlOutNew = fP + g_fCarPosition;
}

void SpeedControlOutput(void)
{
    float fValue;

    /* 速度环 25ms 更新一次，但这里每 5ms/1ms 逐步插值，减少对角度环的冲击。 */
    fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
    g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD
                       + g_fSpeedControlOutOld;
}

void SetMotorVoltageAndDirection(int nLeftMotorPwm, int nRightMotorPwm)
{
    /*
     * 底层电机输出函数：
     * 输入是带符号 PWM，正负表示方向；
     * 本函数负责设置 AIN/BIN 方向引脚，并把 PWM 绝对值写入 TIM3。
     */
    if (nRightMotorPwm < 0)
    {
        /* 右电机反转。 */
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        nRightMotorPwm = -nRightMotorPwm;
    }
    else
    {
        /* 右电机正转。 */
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }

    if (nLeftMotorPwm < 0)
    {
        /* 左电机反转。 */
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        nLeftMotorPwm = -nLeftMotorPwm;
    }
    else
    {
        /* 左电机正转。 */
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }

    /* PWM 最大只能到定时器 ARR 附近，这里统一做输出限幅。 */
    if (nRightMotorPwm > MOTOR_OUT_MAX) nRightMotorPwm = MOTOR_OUT_MAX;
    if (nLeftMotorPwm > MOTOR_OUT_MAX) nLeftMotorPwm = MOTOR_OUT_MAX;

    if (g_cMotorDisable)
    {
        /* 任意保护触发时，PWM 清零，同时让驱动输入进入刹车/禁止状态。 */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    else
    {
        /* TIM3_CH1 对应右轮 PWM，TIM3_CH2 对应左轮 PWM。 */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nRightMotorPwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nLeftMotorPwm);
    }
}

void MotorOutput(void)
{
    /*
     * 最终电机输出合成：
     * 角度环负责“别倒”，速度环负责“别越跑越快”，方向量负责左右差速转向。
     * 左右轮方向量一减一加，就能形成原地或小半径转向。
     */
    g_fLeftMotorOut = g_fAngleControlOut - g_fSpeedControlOut - g_fBluetoothDirection;
    g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection;

    g_fLeftMotorOut = ApplyMotorDeadZone(g_fLeftMotorOut);
    g_fRightMotorOut = ApplyMotorDeadZone(g_fRightMotorOut);

    if ((int)g_fLeftMotorOut > MOTOR_OUT_MAX)    g_fLeftMotorOut = MOTOR_OUT_MAX;
    if ((int)g_fLeftMotorOut < MOTOR_OUT_MIN)    g_fLeftMotorOut = MOTOR_OUT_MIN;
    if ((int)g_fRightMotorOut > MOTOR_OUT_MAX)   g_fRightMotorOut = MOTOR_OUT_MAX;
    if ((int)g_fRightMotorOut < MOTOR_OUT_MIN)   g_fRightMotorOut = MOTOR_OUT_MIN;

    SetMotorVoltageAndDirection((int)g_fLeftMotorOut, (int)g_fRightMotorOut);
}

void AngleControl(void)
{
    /* 角度内环 PD：P 看当前倾角，D 看陀螺仪角速度，是小车能站住的核心。 */
    g_fAngleControlOut = (0.0f - g_fCarAngle) * g_fAngle_P
                       + (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * g_fAngle_D;
}

float Scale(float input, float input_min, float input_max, float output_min, float output_max)
{
    float output;

    /*
     * 比例映射函数：
     * 例如把蓝牙输入 0~10 映射到速度目标 0~50。
     * 它和 LimitFloat 不一样，Scale 会改变数值所在的“量纲/范围”。
     */
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
    float speedTarget;
    float directionTarget;

    /*
     * Steer() 是所有上层控制的统一入口：
     * 蓝牙、超声波、红外循迹都只给 direct/speed，不能直接写 PWM。
     * 这里先把 -10~10 的“人的指令”映射成速度环/转向差速目标。
     */
    if (direct > 0.0f)
    {
        directionTarget = Scale(direct, 0.0f, 10.0f, 0.0f, MANUAL_TURN_OUT_MAX);
    }
    else
    {
        directionTarget = -Scale(direct, 0.0f, -10.0f, 0.0f, MANUAL_TURN_OUT_MAX);
    }

    if (speed > 0.0f)
    {
        speedTarget = Scale(speed, 0.0f, 10.0f, 0.0f, MANUAL_SPEED_OUT_MAX);
    }
    else
    {
        speedTarget = -Scale(speed, 0.0f, -10.0f, 0.0f, MANUAL_SPEED_OUT_MAX);
    }

    speedTarget = LimitFloat(speedTarget, -MANUAL_SPEED_OUT_MAX, MANUAL_SPEED_OUT_MAX);
    directionTarget = LimitFloat(directionTarget, -MANUAL_TURN_OUT_MAX, MANUAL_TURN_OUT_MAX);

    /* 前进/后退切换时削弱旧积分，避免先推不动、动起来后突然冲。 */
    if (((g_fCarSpeedSetTarget > 0.5f) && (speedTarget < -0.5f))
        || ((g_fCarSpeedSetTarget < -0.5f) && (speedTarget > 0.5f)))
    {
        g_fCarPosition *= 0.4f;
    }

    g_fCarSpeedSetTarget = speedTarget;
    g_fBluetoothDirectionTarget = directionTarget;
}

void UltraControl(int mode)
{
    static float fDistanceFilt = ULTRA_FOLLOW_TARGET_CM;
    float fDistanceError;
    float fSpeedCmd;

    if (!IsUltraOK())
    {
        /* 超声波自检失败或测距异常时，不允许继续跟随/避障，先停车。 */
        Steer(0.0f, 0.0f);
        return;
    }

    if (Distance < ULTRA_FOLLOW_NEAR_CM || Distance > ULTRA_FOLLOW_FAR_CM)
    {
        /* 距离太近或太远时，认为数据不适合控制，避免小车乱冲。 */
        Steer(0.0f, 0.0f);
        return;
    }

    /* 对超声波距离做低通滤波，减少手抖和测距跳变带来的速度抖动。 */
    fDistanceFilt = ULTRA_DISTANCE_FILTER_A * fDistanceFilt
                  + (1.0f - ULTRA_DISTANCE_FILTER_A) * Distance;

    if (mode == 0)
    {
        /* 跟随模式：距离大于目标值就前进，距离小于目标值就后退。 */
        fDistanceError = fDistanceFilt - ULTRA_FOLLOW_TARGET_CM;

        if (fDistanceError > -ULTRA_FOLLOW_DEAD_ZONE_CM
            && fDistanceError < ULTRA_FOLLOW_DEAD_ZONE_CM)
        {
            /* 目标距离附近设置死区，否则会因为 1~2cm 的测距波动来回抖。 */
            fSpeedCmd = 0.0f;
        }
        else
        {
            /* 简单 P 控制：距离误差越大，给的前后速度越大。 */
            fSpeedCmd = fDistanceError * ULTRA_FOLLOW_KP;
            if (fSpeedCmd > ULTRA_FOLLOW_SPEED_MAX) fSpeedCmd = ULTRA_FOLLOW_SPEED_MAX;
            if (fSpeedCmd < -ULTRA_FOLLOW_SPEED_MAX) fSpeedCmd = -ULTRA_FOLLOW_SPEED_MAX;
        }

        /* 超声波跟随只控制前后，不控制左右转向。 */
        Steer(0.0f, fSpeedCmd);
    }
    else
    {
        /* 避障模式：15cm 内发现障碍，就设置一段转向编码器计数。 */
        if (Distance > 0.0f && Distance <= 15.0f)
        {
            g_iLeftTurnRoundCnt = 500;
            g_iRightTurnRoundCnt = -500;
        }

        if (g_iLeftTurnRoundCnt > 0 || g_iRightTurnRoundCnt < 0)
        {
            /* 转向计数未完成时继续转。 */
            Steer(5.0f, 0.0f);
        }
        else
        {
            /* 没有障碍或转完以后，继续低速前进。 */
            Steer(0.0f, 4.0f);
        }
    }
}
