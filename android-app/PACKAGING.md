贺群的小车控制器 Android 项目

用途：
这个 App 使用经典蓝牙 SPP，适合 HC-05 / HC-06 蓝牙串口模块。
发送协议和 STM32 小车代码一致：
M 手动
F 前进
B 后退
L 左转
R 右转
S 停止
U 超声波跟随
A 超声波避障

接线：
蓝牙 TX -> STM32 PB11 / USART3_RX
蓝牙 RX -> STM32 PB10 / USART3_TX
GND -> GND
VCC -> 按模块要求接 5V 或 3.3V

打包 APK：
1. 安装 Android Studio
2. 打开本文件夹：贺群的小车控制器_Android
3. 等 Android Studio 自动同步 Gradle
4. 菜单 Build -> Build Bundle(s) / APK(s) -> Build APK(s)
5. APK 通常在 app/build/outputs/apk/debug/app-debug.apk
6. 把 app-debug.apk 用 QQ 发到手机安装

注意：
第一次安装 Android Studio 会下载 Android SDK 和 Gradle 缓存，不会污染 Python 环境。
