#include "manage.h"

/* 固件/EEPROM/MCU 版本信息，主要用于教程版本标识和后续扩展显示。 */
const char FirmwareVer[] = "A53";
const char EEPROMVer[] = "A53";
const char MCUVer[] = "STM32F103";
/* 系统运行状态变量：当前项目里主要保留给后续电量/运行时间管理。 */
uint32_t g_RunTime;
float g_BatVolt;
/* 当前运行模式：蓝牙、超声波、红外任务都会根据它决定谁来调用 Steer()。 */
uint8_t g_CarRunningMode = ULTRA_FOLLOW_MODE;
