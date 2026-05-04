#include "bluetooth.h"
#include "control.h"
#include "manage.h"
#include "usart.h"

uint8_t g_u8BluetoothRxByte;
volatile uint8_t g_u8BluetoothLastByte = 0;
volatile uint32_t g_u32BluetoothRxCount = 0;

void Bluetooth_Init(void)
{
    HAL_UART_Receive_IT(&huart3, &g_u8BluetoothRxByte, 1);
}

void Bluetooth_OnByteReceived(uint8_t data)
{
    g_u8BluetoothLastByte = data;
    g_u32BluetoothRxCount++;

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
        MotorClearAbnormalSpin();
        g_CarRunningMode = CONTROL_MODE;
        Steer(0.0f, 3.0f);
        break;

    case 'B':
    case 'b':
        MotorClearAbnormalSpin();
        g_CarRunningMode = CONTROL_MODE;
        Steer(0.0f, -3.8f);
        break;

    case 'L':
    case 'l':
        MotorClearAbnormalSpin();
        g_CarRunningMode = CONTROL_MODE;
        Steer(-3.2f, 0.0f);
        break;

    case 'R':
    case 'r':
        MotorClearAbnormalSpin();
        g_CarRunningMode = CONTROL_MODE;
        Steer(3.2f, 0.0f);
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

    default:
        break;
    }
}
