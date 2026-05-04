#include "manage.h"

const char FirmwareVer[] = "A53";
const char EEPROMVer[] = "A53";
const char MCUVer[] = "STM32F103";
uint32_t g_RunTime;
float g_BatVolt;
uint8_t g_CarRunningMode = ULTRA_FOLLOW_MODE;
