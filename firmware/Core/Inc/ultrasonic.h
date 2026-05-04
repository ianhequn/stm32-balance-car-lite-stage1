#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"

extern volatile uint8_t TIM1CH4_CAPTURE_STA;
extern volatile uint16_t TIM1CH4_CAPTURE_VAL;
extern float Distance;
extern uint8_t UltraError;

void Read_Distane(void);
void UltraSelfCheck(void);
uint8_t IsUltraOK(void);

#endif
