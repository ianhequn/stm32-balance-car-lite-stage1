#include "infrared.h"
#include "control.h"
//红外模块
/* 四路红外循迹状态位：La/Lb/Rb/Ra 分别代表左外、左内、右内、右外看到黑线。 */
#define IR_LA_BLACK      0x08
#define IR_LB_BLACK      0x04
#define IR_RB_BLACK      0x02
#define IR_RA_BLACK      0x01
//拥有四个红外探头
/*这代表四种状态
0000 = 四个都没看到黑线
0001 = 右外看到黑线
0010 = 右内看到黑线
0100 = 左内看到黑线
1000 = 左外看到黑线
1111 = 四个都看到黑线
*/
/* 循迹控制参数：基础前进速度、丢线速度、转向比例系数和转向限幅。 */
#define TRACE_SPEED_CMD  2.0f
#define TRACE_LOST_SPEED 0.8f
#define TRACE_TURN_KP    1.4f
//转向量 direct = 黑线偏差 error × 比例系数 Kp

#define TRACE_TURN_MAX   6.0f

/* 当前 4 路红外组合状态，OLED/地面站用它观察传感器是否正常。 */
uint8_t g_u8InfraredTraceState;
/* 黑线偏差：负数代表线偏左，正数代表线偏右。 */
int8_t g_iInfraredTraceError;

/* 读取 4 个 GPIO，把红外黑线检测结果压成一个 4 bit 状态值。 */
uint8_t InfraredReadState(void)
	//这里就是把 4 个 GPIO 输入变成一个 4 bit 状态值
{
    uint8_t state = 0;

    /* Black line outputs high level. */
    if (HAL_GPIO_ReadPin(La_GPIO_Port, La_Pin) == GPIO_PIN_SET) state |= IR_LA_BLACK;
    if (HAL_GPIO_ReadPin(Lb_GPIO_Port, Lb_Pin) == GPIO_PIN_SET) state |= IR_LB_BLACK;
    if (HAL_GPIO_ReadPin(Rb_GPIO_Port, Rb_Pin) == GPIO_PIN_SET) state |= IR_RB_BLACK;
    if (HAL_GPIO_ReadPin(Ra_GPIO_Port, Ra_Pin) == GPIO_PIN_SET) state |= IR_RA_BLACK;

    g_u8InfraredTraceState = state;
    return state;
}

/* 红外循迹控制：根据黑线偏差计算 direct，再用固定低速调用 Steer()。 */
void InfraredTraceControl(void)
{
    static int8_t lastError = 0;
    uint8_t state;
    int8_t error = 0;
    float direct;
    float speed = TRACE_SPEED_CMD;

    state = InfraredReadState();

    if (state & IR_LA_BLACK) error -= 3;
    if (state & IR_LB_BLACK) error -= 1;
    if (state & IR_RB_BLACK) error += 1;
    if (state & IR_RA_BLACK) error += 3;

    if (state == 0)
    {
        error = lastError;
        speed = TRACE_LOST_SPEED;
    }
    else if (state == (IR_LA_BLACK | IR_LB_BLACK | IR_RB_BLACK | IR_RA_BLACK))
    {
        error = 0;
        speed = 0.0f;
    }
    else
    {
        lastError = error;
    }

    direct = error * TRACE_TURN_KP;
    if (direct > TRACE_TURN_MAX) direct = TRACE_TURN_MAX;
    if (direct < -TRACE_TURN_MAX) direct = -TRACE_TURN_MAX;

    g_iInfraredTraceError = error;
    Steer(direct, speed);
}
