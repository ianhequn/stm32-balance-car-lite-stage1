#include "ultrasonic.h"
#include "tim.h"

volatile uint8_t TIM1CH4_CAPTURE_STA;
volatile uint16_t TIM1CH4_CAPTURE_VAL;
float Distance;
uint8_t UltraError = 1;

static void UltraDelayUs(uint16_t us)
{
    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) - start) < us)
    {
    }
}

void Read_Distane(void)
{
    uint32_t echo_time;

    if (TIM1CH4_CAPTURE_STA & 0x80)
    {
        echo_time = (uint32_t)(TIM1CH4_CAPTURE_STA & 0x3f) * 65536u
                  + TIM1CH4_CAPTURE_VAL;
        Distance = echo_time * 0.017f;
        UltraError = (Distance > 0.0f && Distance < 450.0f) ? 0 : 1;

        TIM1CH4_CAPTURE_STA = 0;
        TIM1CH4_CAPTURE_VAL = 0;
    }

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    UltraDelayUs(2);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    UltraDelayUs(20);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

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

uint8_t IsUltraOK(void)
{
    return (UltraError == 0);
}
