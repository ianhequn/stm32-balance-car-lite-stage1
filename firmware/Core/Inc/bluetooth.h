#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

/* 蓝牙 USART3 接收状态，供中断和 OLED/调试任务使用。 */
extern uint8_t g_u8BluetoothRxByte;
extern volatile uint8_t g_u8BluetoothLastByte;
extern volatile uint32_t g_u32BluetoothRxCount;
extern volatile uint32_t g_u32BluetoothLastTick;

/* 初始化蓝牙接收；收到字节后的解释逻辑在 Bluetooth_OnByteReceived()。 */
void Bluetooth_Init(void);
void Bluetooth_OnByteReceived(uint8_t data);
void Bluetooth_ManualWatchdog(void);

#endif
