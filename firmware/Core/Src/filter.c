#include "filter.h"

float angle;
float a;

float ComplementaryFilter(float acc, float gyro, float dt)
{
    a = 0.98f;
    /* acc修正长期角度漂移，gyro负责短时间快速响应。 */
    angle = a * (angle + gyro * dt) + (1.0f - a) * acc;
    return angle;
}
