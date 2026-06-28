cmake_minimum_required(VERSION 3.22)
set(CMAKE_TOOLCHAIN_FILE "$ENV{HOME}/stm32-tools/stm32-cmake/cmake/stm32_gcc.cmake")

project(adc-temp C ASM)

# 下载 STM32Cube F1 固件包
stm32_fetch_cube(F1)

# CMSIS + HAL
find_package(CMSIS COMPONENTS STM32F1 REQUIRED)
find_package(HAL COMPONENTS STM32F1 REQUIRED)

# 收集源文件
file(GLOB_RECURSE PROJECT_SOURCES code/src/*.c)

# F1 HAL v1.1.8: stm32-cmake 未单独提供 RCCEx/TIMEx 模块
# ADC 驱动内部调用 HAL_RCCEx_GetPeriphCLKFreq
# TIM 驱动内部调用 HAL_TIMEx_BreakCallback / HAL_TIMEx_CommutCallback
find_file(STM32F1_HAL_RCCEX_SRC
    stm32f1xx_hal_rcc_ex.c
    PATHS ${CMAKE_BINARY_DIR}/_deps/stm32cubef1-src
    PATH_SUFFIXES Drivers/STM32F1xx_HAL_Driver/Src
    REQUIRED
)
find_file(STM32F1_HAL_TIMEX_SRC
    stm32f1xx_hal_tim_ex.c
    PATHS ${CMAKE_BINARY_DIR}/_deps/stm32cubef1-src
    PATH_SUFFIXES Drivers/STM32F1xx_HAL_Driver/Src
    REQUIRED
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${STM32F1_HAL_RCCEX_SRC} ${STM32F1_HAL_TIMEX_SRC})

# 头文件路径
target_include_directories(${PROJECT_NAME} PRIVATE code/include)

# 链接 HAL 模块
target_link_libraries(${PROJECT_NAME} PRIVATE
    CMSIS::STM32::F103CB
    HAL::STM32::F1
    STM32::NoSys
    HAL::STM32::F1::RCC
    HAL::STM32::F1::GPIO
    HAL::STM32::F1::CORTEX
    HAL::STM32::F1::DMA
    HAL::STM32::F1::FLASH
    HAL::STM32::F1::PWR
    HAL::STM32::F1::UART
    HAL::STM32::F1::ADC
    HAL::STM32::F1::TIM
)

# 输出固件大小信息
stm32_print_size_of_target(${PROJECT_NAME})
