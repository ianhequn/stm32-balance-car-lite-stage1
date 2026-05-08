#ifndef __BUTTON_H
#define __BUTTON_H

/* 按键事件标志：1 表示检测到一次有效短按，处理后要清零。 */
extern int g_iButtonState;//声明外部变量，方便其他地方引用

void ButtonScan(void);//声明按键扫描函数

#endif
