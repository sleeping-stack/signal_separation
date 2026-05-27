# 信号分离 —— 2023 年全国大学生电子设计竞赛 H 题解决方案

基于 **STM32H743** 的信号分离项目，使用 CMake + Ninja 构建。

## 项目结构

```
.
├── CMakeLists.txt                    # 顶层 CMake 配置
├── CMakePresets.json                 # CMake 构建预设 (Debug / Release)
├── README.md                         # 项目说明
├── signal_separation.ioc             # STM32CubeMX 工程配置文件
├── startup_stm32h743xx.s             # 汇编启动文件
├── STM32H743XX_FLASH.ld              # 链接脚本 
│
├── cmake/                            # CMake 工具链与子模块（CubeMx生成）
├── Core/                             # 应用核心代码
│   ├── Inc/                          # CubeMX 生成的 HAL 头文件
│   │   ├── adc.h, dma.h, gpio.h, main.h
│   │   ├── spi.h, tim.h, usart.h
│   │   ├── stm32h7xx_hal_conf.h
│   │   └── stm32h7xx_it.h
│   ├── Src/                          # CubeMX 生成的 HAL 源文件
│   │   ├── adc.c, dma.c, gpio.c, main.c
│   │   ├── spi.c, tim.c, usart.c
│   │   ├── stm32h7xx_hal_msp.c, stm32h7xx_it.c
│   │   ├── system_stm32h7xx.c
│   │   └── syscalls.c, sysmem.c
│   └── User/                         # 用户自定义模块
│       ├── Inc/
│       │   ├── ad9833.h              # AD9833 DDS 信号发生器
│       │   ├── adc_dma_timer.h       # ADC + DMA + 定时器采集
│       │   ├── ADS8688.h             # ADS8688 ADC 驱动
│       │   ├── fft.h                 # FFT 频谱分析
│       │   ├── phase_lock_driver.h   # 锁相放大器驱动
│       │   ├── time_utils.h          # 计时/延时工具
│       │   └── uart_screen.h         # 串口屏通信
│       └── Src/
│           ├── ad9833.c
│           ├── adc_dma_timer.c
│           ├── ADS8688.c
│           ├── fft.c
│           ├── phase_lock_driver.c
│           ├── time_utils.c
│           └── uart_screen.c
│
├── Drivers/                          # 硬件驱动与 DSP 库
│   ├── CMSIS/                        # CMSIS 标准接口
│   ├── CMSIS-DSP/                    # ARM CMSIS-DSP 数字信号处理库
│   └── STM32H7xx_HAL_Driver/        
│
├── CMSIS_CORE/                       # CMSIS Core 处理器头文件
│   └── Include/                      # 各 Cortex-M 核心支持头文件
│
└── build/                            # 构建产物目录 (CMake 生成)
    └── Preset-Debug/                 # Debug 构建输出
```

## 环境要求

| 工具 | 用途 |
|------|------|
| CMake (≥ 3.22) | 跨平台构建系统 |
| Ninja | 高性能构建工具 |
| ARM GCC 工具链 (`arm-none-eabi`) | 交叉编译器 |
| STM32CubeMX | 外设初始化代码生成 |

## 快速开始

### 1. 配置构建

```bash
cmake --preset Preset-Debug
```

### 2. 编译

```bash
cmake --build --preset Preset-Debug
```

### 3. 烧录

编译产物位于 `build/Preset-Debug/`，使用 ST-Link、J-Link 等调试器烧录至目标板。

## 开发工具推荐

- **CLion** — JetBrains C/C++ IDE，内置 CMake 和调试支持
- **Visual Studio Code** — 搭配 ST 官方提供的STM32CubeIDE for Visual Studio Code插件