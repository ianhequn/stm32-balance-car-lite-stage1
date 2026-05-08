#include "filter.h"

/* 互补滤波内部保存的上一次角度，下一次滤波会继续沿用。 */
float angle;
/* 滤波权重：越接近 1 越信任陀螺仪短时积分。 */
float a;

/* 互补滤波：acc 给长期绝对角度，gyro 给短时角速度积分。 */
float ComplementaryFilter(float acc, float gyro, float dt)
{
    a = 0.98f;
    /* acc修正长期角度漂移，gyro负责短时间快速响应。 */
    angle = a * (angle + gyro * dt) + (1.0f - a) * acc;
    return angle;
}
