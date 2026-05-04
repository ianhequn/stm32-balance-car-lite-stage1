#ifndef __MANAGE_H
#define __MANAGE_H

#include "main.h"

#define CONTROL_MODE         1
#define INFRARED_TRACE_MODE  2
#define ULTRA_FOLLOW_MODE    3
#define ULTRA_AVOID_MODE     4

extern const char FirmwareVer[];
extern const char EEPROMVer[];
extern const char MCUVer[];
extern uint32_t g_RunTime;
extern float g_BatVolt;
extern uint8_t g_CarRunningMode;

#endif
