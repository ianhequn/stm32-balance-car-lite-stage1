#include "oled.h"
#include "control.h"
#include "manage.h"
#include "ultrasonic.h"
#include "bluetooth.h"
#include "infrared.h"
#include "u8x8.h"
#include <stdio.h>

/* U8x8 显示对象，保存当前 OLED 驱动、字体和通信回调状态。 */
static u8x8_t g_oled;

/* 把运行模式编号转换成屏幕上显示的英文短字符串。 */
static const char *OLED_GetModeString(void)
{
    if (g_CarRunningMode == ULTRA_FOLLOW_MODE) return "FOLLOW";
    if (g_CarRunningMode == ULTRA_AVOID_MODE) return "AVOID ";
    if (g_CarRunningMode == INFRARED_TRACE_MODE) return "TRACE ";
    if (g_CarRunningMode == CONTROL_MODE) return "CTRL  ";
    return "UNKN  ";
}

/* U8x8 软件 SPI 需要的短延时，精度不追求很高，只服务 OLED 时序。 */
static void OLED_Delay10Us(uint16_t n)
{
    volatile uint16_t i;
    volatile uint16_t j;

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < 90; j++)
        {
        }
    }
}

/* U8x8 的 HAL 适配回调：库通过 msg 请求我们拉高/拉低 GPIO 或延时。 */
uint8_t OLED_U8x8GpioAndDelay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;

    switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET);
        break;

    case U8X8_MSG_DELAY_MILLI:
        HAL_Delay(arg_int);
        break;

    case U8X8_MSG_DELAY_10MICRO:
        OLED_Delay10Us(arg_int);
        break;

    case U8X8_MSG_DELAY_100NANO:
    case U8X8_MSG_DELAY_NANO:
        break;

    case U8X8_MSG_GPIO_SPI_CLOCK:
        HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin,
                          arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;

    case U8X8_MSG_GPIO_SPI_DATA:
        HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin,
                          arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;

    case U8X8_MSG_GPIO_CS:
        break;

    case U8X8_MSG_GPIO_DC:
        HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin,
                          arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;

    case U8X8_MSG_GPIO_RESET:
        HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin,
                          arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;

    default:
        return 0;
    }

    return 1;
}

/* 初始化 SSD1306 128x64 OLED，选择 4 线软件 SPI 和文本字体。 */
void OLED_Init(void)
{
    u8x8_Setup(&g_oled,
               u8x8_d_ssd1306_128x64_noname,
               u8x8_cad_001,
               u8x8_byte_4wire_sw_spi,
               OLED_U8x8GpioAndDelay);
    u8x8_InitDisplay(&g_oled);
    u8x8_SetPowerSave(&g_oled, 0);
    u8x8_SetFont(&g_oled, u8x8_font_chroma48medium8_r);
    u8x8_ClearDisplay(&g_oled);
    u8x8_DrawString(&g_oled, 0, 0, "Hello OLED");
}

/* OLED 主页刷新：显示模式、角度、距离、速度、零点、蓝牙和红外状态。 */
void OLED_ShowHomePage(void)
{
    char line[18];
    char bt;

    u8x8_ClearDisplay(&g_oled);

    snprintf(line, sizeof(line), "MODE:%s", OLED_GetModeString());
    u8x8_DrawString(&g_oled, 0, 0, line);

    snprintf(line, sizeof(line), "ANG :%6.1f", g_fCarAngle);
    u8x8_DrawString(&g_oled, 0, 1, line);

    snprintf(line, sizeof(line), "DIS :%6.1fcm", Distance);
    u8x8_DrawString(&g_oled, 0, 2, line);

    snprintf(line, sizeof(line), "SPD :%6.1f", g_fCarSpeedSet);
    u8x8_DrawString(&g_oled, 0, 3, line);

    snprintf(line, sizeof(line), "ZRO :%6.1f", g_fCarAngleOffset);
    u8x8_DrawString(&g_oled, 0, 4, line);

    bt = (g_u8BluetoothLastByte == 0) ? '-' : (char)g_u8BluetoothLastByte;
    snprintf(line, sizeof(line), "BT  :%c %5lu", bt, (unsigned long)g_u32BluetoothRxCount);
    u8x8_DrawString(&g_oled, 0, 5, line);

    snprintf(line, sizeof(line), "IR  :%02X %4d", g_u8InfraredTraceState, g_iInfraredTraceError);
    u8x8_DrawString(&g_oled, 0, 6, line);
}
