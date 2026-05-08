#ifndef __FILTER_H
#define __FILTER_H

/* 输入加速度角度、陀螺仪角速度和周期 dt，输出融合后的车身角度。 */
float ComplementaryFilter(float acc, float gyro, float dt);

#endif
