#include "ultrasonic.h"
#include "tim.h"
//超声波模块，用来测距离的
/* TIM1_CH4 输入捕获状态：bit7 表示捕获完成，bit6 表示正在测高电平，低 6 位记录溢出次数。 */
volatile uint8_t TIM1CH4_CAPTURE_STA;
/* Echo 下降沿捕获到的计数值，配合溢出次数得到高电平总时间。 */
volatile uint16_t TIM1CH4_CAPTURE_VAL;
/* 超声波测出的距离，单位 cm，控制和 OLED 都读取这个变量。 */
float Distance;
/* 超声波错误标志：0 正常，1 异常或距离超出硬件有效范围。 */
uint8_t UltraError = 1;

/* 简单 us 级延时，用 TIM1 当前计数产生 TRIG 触发脉冲。 */
static void UltraDelayUs(uint16_t us)
{
    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) - start) < us)
    {
    }
}

/* 读取上一次 Echo 捕获结果，并触发下一次 HC-SR04 测距。 */
void Read_Distane(void)
{
    uint32_t echo_time;

    if (TIM1CH4_CAPTURE_STA & 0x80)
    {
        echo_time = (uint32_t)(TIM1CH4_CAPTURE_STA & 0x3f) * 65536u
                  + TIM1CH4_CAPTURE_VAL;
        Distance = echo_time * 0.017f;
			//距离 = 时间 × 0.034 / 2= 时间 × 0.017
      //声速是  340 m/s = 0.034 cm/us
			UltraError = (Distance > 0.0f && Distance < 450.0f) ? 0 : 1;
/*0cm < Distance < 450cm  -> 认为超声波正常，UltraError = 0
Distance <= 0cm         -> 异常，UltraError = 1
Distance >= 450cm       -> 异常，UltraError = 1 */
        TIM1CH4_CAPTURE_STA = 0;
        TIM1CH4_CAPTURE_VAL = 0;
    }

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    UltraDelayUs(2);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    UltraDelayUs(20);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

/* 上电自检：最多尝试 5 次测距，只要一次正常就认为超声波可用。 */
void UltraSelfCheck(void)
{
    uint8_t i;

    UltraError = 1;
    for (i = 0; i < 5; i++)
    {
        Read_Distane();
        HAL_Delay(30);
        Read_Distane();
        if (IsUltraOK())
        {
            break;
        }
    }
}
//看看超声波的测的距离是不是正常的
/* 查询超声波是否正常，给 UltraControl() 做安全判断。 */
uint8_t IsUltraOK(void)
{
    return (UltraError == 0);
}
