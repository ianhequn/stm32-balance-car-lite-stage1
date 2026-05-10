#include "bluetooth.h"
#include "control.h"
#include "manage.h"
#include "usart.h"
//该模块负责收命令
/* USART3 中断接收缓冲区，每次只接收 1 个蓝牙字符。 */
uint8_t g_u8BluetoothRxByte;
/* 最近一次收到的蓝牙命令，用于 OLED/地面站调试显示。 */
volatile uint8_t g_u8BluetoothLastByte = 0;
/* 蓝牙接收计数，按键没有反应时可以用它判断 STM32 是否真的收到数据。 */
volatile uint32_t g_u32BluetoothRxCount = 0;
/* 最近一次蓝牙命令时间，用来做手动模式超时刹停，防止松手 S 丢包后小车继续跑。 */
volatile uint32_t g_u32BluetoothLastTick = 0;

#define BLUETOOTH_MANUAL_TIMEOUT_MS  350u

/* 启动 USART3 单字节中断接收，后续每收到 1 字节都会进回调。 */
void Bluetooth_Init(void)
{
    HAL_UART_Receive_IT(&huart3, &g_u8BluetoothRxByte, 1);
}
//逻辑就是按照接受的字符是什么来判断
void Bluetooth_OnByteReceived(uint8_t data)
{
    g_u8BluetoothLastByte = data;
    g_u32BluetoothRxCount++;
    g_u32BluetoothLastTick = HAL_GetTick();

    /* 单字符协议：手机 App 每发一个字符，小车切模式或更新 Steer 目标。 */
    switch (data)
    {
    case 'M':
    case 'm':
        MotorClearAbnormalSpin();
        g_CarRunningMode = CONTROL_MODE;
        Steer(0.0f, 0.0f);
        break;

    case 'F':
    case 'f':
        MotorResumeFromProtection();
        g_CarRunningMode = CONTROL_MODE;
        /* 第一阶段展示先压低前进命令，减少速度起来后继续冲。 */
        Steer(0.0f, 2.6f);
        break;

    case 'B':
    case 'b':
        MotorResumeFromProtection();
        g_CarRunningMode = CONTROL_MODE;
        /* 后退命令也保守一点，让 S/反向响应更容易控制。 */
        Steer(0.0f, -3.2f);
        break;

    case 'L':
    case 'l':
        MotorResumeFromProtection();
        g_CarRunningMode = CONTROL_MODE;
        /* 降低原地转向幅度，避免小车静止附近左右抽动太明显。 */
        Steer(-2.8f, 0.0f);
        break;

    case 'R':
    case 'r':
        MotorResumeFromProtection();
        g_CarRunningMode = CONTROL_MODE;
        /* 与左转保持对称，方便调试时判断方向和手感。 */
        Steer(2.8f, 0.0f);
        break;

    case 'S':
    case 's':
        MotorClearAbnormalSpin();
        Steer(0.0f, 0.0f);
        break;

    case 'U':
    case 'u':
        MotorClearAbnormalSpin();
        g_CarRunningMode = ULTRA_FOLLOW_MODE;
        Steer(0.0f, 0.0f);
        break;

    case 'A':
    case 'a':
        MotorClearAbnormalSpin();
        g_CarRunningMode = ULTRA_AVOID_MODE;
        Steer(0.0f, 0.0f);
        break;

    case 'T':
    case 't':
        MotorClearAbnormalSpin();
        g_CarRunningMode = INFRARED_TRACE_MODE;
        Steer(0.0f, 0.0f);
        break;

    default:
        break;
    }
}

void Bluetooth_ManualWatchdog(void)
{
    if (g_CarRunningMode != CONTROL_MODE)
    {
        return;
    }

    if ((HAL_GetTick() - g_u32BluetoothLastTick) > BLUETOOTH_MANUAL_TIMEOUT_MS)
    {
        /*
         * 手动模式空闲保护：
         * 如果手机松手时 S 指令因为蓝牙瞬断丢了，这里会自动清目标，
         * 避免 CTRL 模式下保留上一次 F/B/L/R 导致小车一直跑。
         */
        MotorClearAbnormalSpin();
        Steer(0.0f, 0.0f);
        g_u32BluetoothLastTick = HAL_GetTick();
    }
}
