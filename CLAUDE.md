# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

STM32F103CBT6 ADC 温度采集程序，基于 HAL 库 + stm32-cmake 构建。
- ADC1_IN6 (PA6) 连续采样 → 温度计算 → USART1 每秒上报
- 3 路共阳极 LED (PA8/PB14/PB12) 间隔 1 秒闪烁
- 外部晶振 10MHz → PLL 70MHz

## 构建命令

```bash
# 配置 (首次/修改 CMakeLists.txt 后)
cmake -S . -B build -G Ninja

# 编译
cmake --build build

# 烧录 (需 ST-Link 连接)
arm-none-eabi-objcopy -O binary build/adc-temp.elf build/adc-temp.bin
st-flash --reset write build/adc-temp.bin 0x08000000
```

## 工具链

所有工具位于 `C:\msys64\mingw64\bin`，已写入 Windows 用户 PATH 和 `~/.bashrc`：
- `arm-none-eabi-gcc` 13.3.0 (通过 MSYS2 pacman 安装)
- `cmake` + `ninja` + `st-flash`/`st-info`

stm32-cmake 框架位于 `~/stm32-tools/stm32-cmake/` (HOME = `/d/Cadence/SPB_16.6/pcbenv/`)。

## 架构

```
code/
├── include/
│   └── stm32f1xx_hal_conf.h    # HAL 模块使能 + HSE=10MHz
└── src/
    └── main.c                  # 单一源文件，所有逻辑
CMakeLists.txt                  # stm32-cmake 构建配置
```

`main.c` 结构：`HAL_Init()` → `SystemClock_Config()` → 各外设 Init → 主循环 (`while(1)` 每秒读 ADC → 算温度 → 串口发 → LED 翻转 → `HAL_Delay(1000)`)。

## 关键技术细节

### CMakeLists.txt 中的设备命名
- `find_package(CMSIS COMPONENTS STM32F1 REQUIRED)` — 家族级
- `CMSIS::STM32::F103CB` — 设备级 (stm32-cmake 用 `F103CB` 而非 `F103CBT6` 或 `F103xB`)

### RCCEx 手动编译 (关键陷阱)
stm32-cmake 的 F1 家族没有独立的 `HAL::STM32::F1::RCCEx` 模块，但 HAL ADC 驱动内部调用 `HAL_RCCEx_GetPeriphCLKFreq()`。CMakeLists.txt 通过 `find_file()` 手动将 `stm32f1xx_hal_rcc_ex.c` 加入编译。**修改外设后务必检查是否触发新的 RCCEx 依赖。**

### USART1 重映射
使用 PB6/PB7 而非默认的 PA9/PA10，必须：
1. `__HAL_RCC_AFIO_CLK_ENABLE()` — AFIO 时钟
2. `__HAL_AFIO_REMAP_USART1_ENABLE()` — 重映射宏
3. PB6 配为 `GPIO_MODE_AF_PP`，PB7 配为 `GPIO_MODE_INPUT` + `GPIO_PULLUP`

### 共阳极 LED
`GPIO_PIN_RESET` (低电平) = LED 点亮，`GPIO_PIN_SET` (高电平) = LED 熄灭。

### 时钟与 SysTick
STM32F103 最高 72MHz，10MHz 晶振无法整除：`HSE / 2 (PLLXTPRE) × 14 (PLLMUL) = 70MHz`。
`HAL_Init()` 按默认 HSI 8MHz 配 SysTick，`SystemClock_Config()` 后需手动调用 `HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U)` 更新时基。

### 温度公式
```
V_adc = ADC码值 / 4096 × 3.3
温度  = ((V_adc / 17.47 / 0.625) × 1000 - 100) / 0.385
```
- ADC 参考电压 3.3V，12 位分辨率
- 温度范围估计 0–50°C，对应 ADC 约 1355–1616

## 串口输出格式

```
ADC码值：1486。温度：25.0。
```

115200-8N1，每秒一条，`\r\n` 换行。
