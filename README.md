# STM32 两轮自平衡小车 Stage 1

这是一个基于 STM32F103 的两轮自平衡小车项目，硬件平台为喵呜实验室「小霸王 Lite」增强版套件。当前仓库记录第一阶段成果：小车能够完成姿态采集、角度环与速度环控制、超声波跟随/避障、OLED 状态显示，以及 Android 手机蓝牙遥控。

项目不是只按教程堆功能，而是在教程基础上补了调试可视化、安全保护、手机 APK 和项目文档整理，目标是形成一个可以讲清楚、可以演示、可以继续升级的嵌入式系统作品。

## 当前成果

- MPU6050 姿态采集与互补滤波
- 编码器脉冲读取，左右轮速度反馈
- 角度环 PD 控制与速度外环 PI 控制
- 机械零点、提起/着陆识别、电机保护
- HC-SR04 超声波测距、跟随与避障模式
- SSD1306 OLED 状态显示
- USART3 蓝牙单字符遥控协议
- Android 蓝牙遥控 APK
- 喵呜地面站串口曲线调试输出

## 仓库结构

```text
firmware/       STM32CubeMX + Keil 固件工程
android-app/    Android 蓝牙遥控器源码
release/apk/    可直接安装的 APK
media/videos/   第一阶段演示视频
media/screenshots/ App 截图
docs/           项目理解、控制链路和测试说明
```

## 硬件平台

- STM32F103 核心板
- MPU6050 姿态传感器
- TB6612FNG 电机驱动
- 两个带编码器直流减速电机
- HC-SR04 超声波模块
- SSD1306 0.96 寸 OLED
- 经典蓝牙串口模块 HC-05 / HC-06 / MiaowLabs 蓝牙模块
- 7.4V 锂电池供电

## 固件工程

Keil 工程路径：

```text
firmware/MDK-ARM/a.uvprojx
```

最近一次本地编译结果：

```text
0 Error(s), 0 Warning(s)
```

主要源码模块：

```text
Core/Src/control.c      角度环、速度环、电机输出、转向和安全保护
Core/Src/bluetooth.c    蓝牙串口命令解析
Core/Src/ultrasonic.c   超声波测距
Core/Src/oled.c         OLED 状态页
Core/Src/manage.c       运行模式管理
Core/Src/stm32f1xx_it.c SysTick、TIM1、USART3 中断回调
```

## Android 遥控器

APK 文件：

```text
release/apk/hequn-car-controller.apk
```

App 名称：

```text
贺群的小车控制器
```

蓝牙协议：

```text
M = 手动遥控模式
F = 前进
B = 后退
L = 左转
R = 右转
S = 停止
U = 超声波跟随模式
A = 超声波避障模式
```

当前 App 的方向键逻辑是“按住才运动，松手自动发送停止”，这样比点按式遥控更适合演示，也能避免小车一直保持速度目标。

## 演示素材

视频：

```text
media/videos/demo-balance-and-control-1.mp4
media/videos/demo-balance-and-control-2.mp4
```

截图：

```text
media/screenshots/app-connected.jpg
media/screenshots/app-log.jpg
```

## 当前不足

第一阶段先到“能站、能测、能遥控、能展示”。当前速度环和角度环还没有做最终精调，现象包括：

- 静止时仍有轻微抖动
- 遥控速度和刹车手感还需要继续调
- 超声波跟随效果可用，但还不够平滑
- App 目前是最小可用版，还没有速度档位和参数调节

这些问题保留到第二阶段继续做，不影响第一阶段作为嵌入式系统集成作品归档。

## Notion 项目理解

这个项目对应的学习链路是：

1. CubeMX 生成 STM32 工程
2. GPIO、按键、定时器和串口基础
3. 编码器与 PWM 电机驱动
4. MPU6050 姿态采集与互补滤波
5. PID 基础、角度环、速度环
6. 机械零点、提起/着陆识别
7. 超声波测距、避障、跟随
8. OLED 状态显示
9. 蓝牙遥控与 Android App

更详细的整理见：

```text
docs/project-summary.md
docs/control-and-debug-notes.md
docs/bluetooth-app-notes.md
docs/learning-reflection.md
```

## 简历表达

基于 STM32F103 的两轮自平衡小车系统，完成 MPU6050 姿态解算、编码器速度反馈、角度 PD 与速度 PI 闭环控制、超声波跟随/避障、OLED 状态显示和 Android 蓝牙遥控 App，实现从底层驱动、闭环控制到移动端交互的完整嵌入式系统闭环。
