#ifndef __INFRARED_H
#define __INFRARED_H

#include "main.h"

/* 红外循迹状态和误差，供 OLED/地面站显示。 */
extern uint8_t g_u8InfraredTraceState;
extern int8_t g_iInfraredTraceError;

/* 读取红外状态；InfraredTraceControl() 会进一步转换成 Steer 指令。 */
uint8_t InfraredReadState(void);
void InfraredTraceControl(void);

#endif
