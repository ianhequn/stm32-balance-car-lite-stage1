# 控制与调试说明

## 控制链路

小车控制可以按两个闭环理解：

```text
角度环：车身角度 -> 角度 PD -> 电机基础输出
速度环：编码器速度 -> 速度 PI -> 修正车体前后移动趋势
```

最终电机输出：

```c
left  = angle_output - speed_output - turn_output;
right = angle_output - speed_output + turn_output;
```

其中 `turn_output` 来自蓝牙遥控或后续自动模式，用左右轮差速实现转向。

## 关键变量

```text
g_fCarAngle            当前车身角度
g_fGyroAngleSpeed      当前角速度
g_fAngleControlOut     角度环输出
g_fCarSpeed            编码器估算速度
g_fCarSpeedSet         速度目标
g_fSpeedControlOut     速度环输出
g_fBluetoothDirection  蓝牙转向差速
Distance               超声波距离
g_CarRunningMode       小车运行模式
```

## 调试输出

串口地面站使用 `$...;` 格式连续输出多通道数据，用于观察：

- 角度
- 超声波距离
- 速度目标
- 速度环输出
- 蓝牙最后接收字符
- 蓝牙接收计数

OLED 显示：

```text
MODE:CTRL
ANG : -2.3
DIS : 18.5cm
SPD : 12.0
BT  :F     3
```

## 目前调参状态

当前参数是第一阶段临时演示参数，不是最终最优参数。

当前已知问题：

- 静止时还有轻微抖动
- 速度快时仍可能有继续加速感
- 停止和后退手感仍需二阶段继续调

下一阶段建议优先顺序：

1. 先调机械零点 `CAR_ZERO_ANGLE`
2. 再调角度环 `g_fAngle_P / g_fAngle_D`
3. 再调速度环 `g_fSpeed_P / g_fSpeed_I`
4. 最后调蓝牙速度档位和 App 交互

不要一开始就同时改很多参数，否则无法判断是哪一项产生效果。
