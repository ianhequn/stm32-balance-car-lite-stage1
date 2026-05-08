#ifndef __MANAGE_H
#define __MANAGE_H

#include "main.h"

/* 小车运行模式编号：只允许一个上层模式在当前周期写入运动目标。 */
#define CONTROL_MODE         1
#define INFRARED_TRACE_MODE  2
#define ULTRA_FOLLOW_MODE    3
#define ULTRA_AVOID_MODE     4

/* 版本与运行模式变量在 OLED、蓝牙、FreeRTOS 任务中会被引用。 */
extern const char FirmwareVer[];
extern const char EEPROMVer[];
extern const char MCUVer[];
extern uint32_t g_RunTime;
extern float g_BatVolt;
extern uint8_t g_CarRunningMode;

#endif
