# 蓝牙遥控与 Android App

## 蓝牙硬件连接

```text
蓝牙 TX -> STM32 PB11 / USART3_RX
蓝牙 RX -> STM32 PB10 / USART3_TX
蓝牙 GND -> STM32 GND
蓝牙 VCC -> 5V 或 3.3V
```

串口参数：

```text
115200 / 8N1
```

常见配对码：

```text
1234
0000
```

## 命令协议

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

## 固件接收逻辑

USART3 使用中断接收 1 字节：

```c
HAL_UART_Receive_IT(&huart3, &g_u8BluetoothRxByte, 1);
```

接收完成后进入回调，解析命令并重新开启下一次接收。

## App 逻辑

App 使用经典蓝牙 SPP 连接 HC-05/HC-06 类模块。

方向键采用：

```text
按下：发送 F/B/L/R
松手：发送 S
```

这样比“点一下持续运动”更适合演示，也更容易保证安全。

## APK

```text
release/apk/hequn-car-controller.apk
```

安装后先在系统蓝牙里完成配对，再打开 App 连接已配对设备。
