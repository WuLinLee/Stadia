# Stadia Haptic Bridge

一个面向 ESP32-S3 原生 USB + BLE 的独立实验项目：让电脑端通过 Xbox 360/XInput 兼容 USB 手柄发送游戏震动，把震动命令映射为 Stadia 蓝牙手柄的 HID over GATT 输出。

本仓库是独立实现，不从 stadia-dongle-esp-2 复制源码。参考项目只用于确认使用场景与公开协议事实；USB/GATT 桥接、映射层、测试和网页由本仓库单独组织。

## 设计目标

- 默认 **Faithful / 线性参考** 映射：Xbox 360 的每路 8 位强度 x 转成 Stadia 的 16 位小端强度 x * 257。等比例扩展，两个端点精确到 0 和 65535，不凭空推断 Google 云端算法。
- 同时提供两个可选的温和非线性曲线（Gentle、Punch），保留零点和满量程，低/中强度映射不同。它们是可测试的实验配置，不声称是 Google 原版曲线。
- GATT Write-With-Response 串行化；强度变化合并为最新目标，但停止震动（零值）也必须送达；失败时有界退避重试。
- GitHub Actions 负责 ESP-IDF 云编译、主机端映射公式测试及固件 Artifact；可选地部署在线刷机/曲线演示页面。

## 构建和固件变体

GitHub Actions 对每次 push / PR 编译三个版本：
- firmware-reference-merged.bin：精确线性映射，建议作为验证基线
- firmware-gentle-merged.bin：轻微低/中强度增强
- firmware-punch-merged.bin：更明显的低/中强度增强

从仓库 Actions 的对应运行记录下载 Artifact。打包文件是 ESP32-S3 的完整刷写镜像，使用 ESP Web Tools 或 esptool 从偏移 0 刷写；不要把合并镜像当作 OTA 应用分区镜像。

网页位于 docs/。将仓库 Pages 配置完成后，在仓库 Actions Variables 中设置 ENABLE_GITHUB_PAGES=true 才会执行部署；默认不改变仓库可见性，也不要求 Pages 功能必须启用。

## 硬件与限制

- 目标：ESP32-S3，原生 USB OTG 连接电脑，BLE Central 连接已切换到 Bluetooth 模式并完成配对准备的 Stadia 控制器。
- USB 输入端暴露 Xbox 360 风格的控制器和震动输出报告；固件接收游戏/系统送来的两路强弱马达等级，并通过 Stadia BLE HID 报告输出。
- 此最小版本专注单个 Stadia 手柄和两路主马达；不包含原仓库的按键扩展、多个控制器、Wi-Fi 配置/OTA、鼠标模式或其他附属功能。
- Stadia 云游戏时期 Google 云端的原始震动报文与完整映射曲线没有在本研究中得到证实。任何“更还原”的结论都需要原版抓包或者实际加速度测量；本项目提供明确公式和可复现实验配置，而非冒称复刻 Google 私有算法。

## 验证状态

Actions 绿勾只证明主机端公式测试与固件编译完成。Windows XInput 枚举、实际 BLE 配对、游戏震动及马达物理响应仍需在 ESP32-S3 + Stadia 手柄上验证。
