# Self-Balancing Car

基于 STM32F103 的双轮自平衡车，包含姿态估计、电机及编码器控制、位置保持、任务控制和本地调参面板。

## Project Status

当前源码对应 2026-11-13 的 `Competition_StopHold` 版本。该版本已烧录并通过程序读回核验。平衡、停车、任务、避障、显示、串口等功能均已完成实车验证，停车效果良好；该版本现已冻结。具体测试数据未在此记录。

原工程历史记录：四个 Keil 目标 0 错误、0 警告；固件模拟检查 367 项、面板检查 42 项通过。并非本仓库环境重新验证的结果。

## Hardware

STM32F103C8T6、MPU6050、D153C 电机驱动和带编码器的双轮电机，另有按键、LED、OLED 与 VL53L1X 接口。参见 [接线和操作说明](docs/)。

## Repository Layout

- `firmware/`：固件、Keil 工程、模拟测试和构建脚本。
- `panel/`：Node.js 本地调参面板与固件身份清单。
- `docs/`：接线、按键和停车操作说明。记录冻结版的接线与操作。

## Build and Test

固件使用 Keil MDK/Arm Compiler 5 和 STM32F1 器件支持包。厂商库、Flash 算法、SVD 与工具链未随仓库分发；需自行从合法来源取得对应版本，放入 `firmware/Libraries/` 中与工程引用一致的路径。工程引用包括 `Libraries/Device`、`Libraries/CMSIS` 和 `Libraries/Startup/startup_stm32f10x_md.s`。调试配置可能还需要 Flash 算法与 SVD。其他版本未必能复现原始构建。

在 Windows PowerShell 7 中，根据安装位置运行：

```powershell
cd firmware
New-Item -ItemType Directory -Force evidence, release | Out-Null
./build.ps1 -KeilRoot 'C:/Keil_v5' -Target All
./scripts/run_regression.ps1 -KeilRoot 'C:/Keil_v5' -Suite Parking
./scripts/run_regression.ps1 -KeilRoot 'C:/Keil_v5' -Suite Review
./scripts/run_regression.ps1 -KeilRoot 'C:/Keil_v5' -Suite Stop
```

面板使用 Node.js。连接设备前安装 OpenOCD，设置 `OPENOCD_SCRIPTS` 为其 scripts 目录；若可执行文件不在 PATH 中，再设置 `OPENOCD_EXE` 为可执行文件完整路径。进入 `panel/` 运行 `npm start`，使用控制台输出的本地地址打开面板。面板以 Flash 摘要匹配固件和符号地址；重新编译得到不同固件时，现有清单不能直接用于新固件。

`npm test` 包含依赖历史版本二进制及 map 文件的身份检查；仓库未包含这些构建产物，因此完整测试需要另行准备相应历史工程和产物。

## License

项目原创内容采用 [MIT License](LICENSE)。第三方软件和器件支持文件各自遵循其原有许可，不包含在该 MIT 授权中。

