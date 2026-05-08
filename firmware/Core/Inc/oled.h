#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* OLED 只负责状态显示，不参与平衡控制。 */
void OLED_Init(void);
void OLED_ShowHomePage(void);

#endif
