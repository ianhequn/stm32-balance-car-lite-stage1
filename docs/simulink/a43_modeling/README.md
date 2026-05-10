# A43 数学建模验证工程

这个目录按喵呜实验室 A43「数学建模」文章搭建，用来验证：

```text
两轮自平衡小车可以简化为倒立摆；
无控制时系统不稳定；
加入角度 P 和角速度 D 后，系统具备稳定可行性。
```

## 模型依据

A43 文章把小车近似为高度为 `L`、质量为 `m` 的倒立摆，开环传递函数核心形式是：

```text
H(s) = 1 / (s^2 - g/L)
```

它的极点为：

```text
s = ±sqrt(g/L)
```

其中一个极点在右半平面，所以无控制时天然不稳定。

加入 PD 后，闭环分母变为：

```text
s^2 + (k2/L)s + (k1-g)/L
```

稳定条件是：

```text
k1 > g
k2 > 0
```

这对应代码里的角度环：

```c
g_fAngleControlOut =
    (0.0f - g_fCarAngle) * g_fAngle_P
  + (0.0f - g_fGyroAngleSpeed) * g_fAngle_D;
```

## 文件说明

- `a43_params.m`
  - 放 `g、L、k1、k2` 和扰动参数。

- `build_a43_simulink_model.m`
  - 自动生成 Simulink 模型 `a43_balance_model_verification.slx`。

- `run_a43_verification.m`
  - 一键运行仿真，输出角度响应图和极点图。

## 怎么运行

在 MATLAB 命令行执行：

```matlab
cd('D:\cube\a\simulink_balance_car\a43_modeling')
run_a43_verification
```

运行后会生成：

- `a43_balance_model_verification.slx`
- `a43_balance_model_verification_result.png`
- `a43_pole_map.png`

## 结果怎么看

第一张图：

```text
第一行：扰动脉冲，相当于轻推小车；
第二行：无控制开环角度，会发散；
第三行：加入 PD 后，角度会恢复到稳定附近。
```

第二张图：

```text
红叉：开环极点，有一个在右半平面；
蓝圈：PD 后的闭环极点，都在左半平面。
```

这就完成了 A43 文章想证明的事：

```text
平衡车本身不稳定，但在角度和角速度反馈下，理论上具备平衡可行性。
```
