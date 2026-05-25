# 本项目为电赛23年h题解决方案

基于 STM32H743 的信号分离项目，使用 CMake + Ninja 构建。

## 项目结构

```
.
|-- CMakeLists.txt                # 顶层 CMake 配置
|-- CMakePresets.json             # 预设构建配置 (Preset-Debug/Release)
|-- signal_separation.ioc         # STM32CubeMX 工程配置
|-- STM32H743XX_FLASH.ld          # 链接脚本
|-- startup_stm32h743xx.s         # 启动文件
|-- cmake/                        # 工具链与 CubeMX 生成的 CMake 配置
|-- Core/                         # HAL 初始化与用户代码
|   |-- Inc/                      # CubeMX 生成的头文件
|   |-- Src/                      # CubeMX 生成的源文件
|   `-- User/                     # 用户自定义模块
|       |-- Inc/
|       |   |-- ad9833.h
|       |   |-- adc_dma_timer.h
|       |   |-- ADS8688.h
|       |   |-- fft.h
|       |   |-- phase_lock_driver.h
|       |   |-- time_utils.h
|       |   `-- uart_screen.h
|       `-- Src/
|           |-- ad9833.c          # AD9833 信号发生器驱动
|           |-- adc_dma_timer.c   # ADC + DMA + 定时器采集配置
|           |-- ADS8688.c         # ADS8688 模数转换器驱动
|           |-- fft.c             # FFT 频谱分析
|           |-- phase_lock_driver.c # 锁相放大器相关逻辑
|           |-- time_utils.c      # 计时工具
|           `-- uart_screen.c     # 串口屏通信
|-- Drivers/                      # HAL/CMSIS/CMSIS-DSP 驱动与库
`-- build/                        # 构建产物目录 (由 CMake 生成)
```

## 环境要求

- CMake
- Ninja
- ARM GCC 工具链 (arm-none-eabi)

## 如何构建

使用 CMake 进行配置与编译，推荐使用clion，STM32CubeIDE for Visual Studio Code 等工具进行开发。

构建产物默认在 build/Preset-Debug/ 下生成。
