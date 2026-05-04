#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

extern uint8_t g_u8BluetoothRxByte;
extern volatile uint8_t g_u8BluetoothLastByte;
extern volatile uint32_t g_u32BluetoothRxCount;

void Bluetooth_Init(void);
void Bluetooth_OnByteReceived(uint8_t data);

#endif
