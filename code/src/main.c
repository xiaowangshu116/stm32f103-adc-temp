/**
  ******************************************************************************
  * @file    main.c
  * @brief   STM32F103CBT6 ADC温度采集 + 3路LED + 串口上报
  *          外部晶振: 10MHz → PLL 70MHz
  *          LED:  PA8, PB14, PB12 (共阳极, 低电平点亮)
  *          ADC:  ADC1_IN6 (PA6), 12位
  *          UART: USART1 重映射至 PB6(TX)/PB7(RX), 115200-8N1
  ******************************************************************************
  */

#include "stm32f1xx_hal.h"
#include <stdio.h>

/* ====================== 外设句柄 ====================== */
ADC_HandleTypeDef   hadc1;
UART_HandleTypeDef  huart1;

/* ====================== 函数声明 ====================== */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_Init(void);
void Error_Handler(void);

/* ====================== LED 引脚定义 ====================== */
#define LED1_PIN        GPIO_PIN_8
#define LED1_PORT       GPIOA
#define LED2_PIN        GPIO_PIN_14
#define LED2_PORT       GPIOB
#define LED3_PIN        GPIO_PIN_12
#define LED3_PORT       GPIOB

/* 共阳极: 低电平=亮, 高电平=灭 */
#define LED_ON(port, pin)   HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define LED_OFF(port, pin)  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define LED_TOGGLE(port, pin) HAL_GPIO_TogglePin(port, pin)

/* ====================== SysTick 中断处理 ====================== */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ====================== 系统时钟配置 ====================== */
/**
  * @brief  HSE 10MHz → PLL 70MHz
  *         HSE → PLLXTPRE(/2) → 5MHz → PLLMUL(×14) → 70MHz
  *         HCLK = 70MHz, PCLK1 = 35MHz, PCLK2 = 70MHz
  */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* --- 1. HSE + PLL 配置 --- */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;  /* HSE/2 = 5MHz */
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL14;        /* 5×14 = 70MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* --- 2. 总线 + Flash 延时配置 --- */
    /* FLASH_LATENCY_2: 48MHz < SYSCLK ≤ 72MHz 需要 2 个等待周期 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;       /* HCLK  = 70MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;         /* PCLK1 = 35MHz (≤36MHz) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         /* PCLK2 = 70MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    /* --- 3. 重配 SysTick (HAL_Init 按 8MHz 配置，现改为 70MHz) --- */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/* ====================== GPIO 初始化 (LED) ====================== */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能 GPIOA, GPIOB 时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA8, PB14, PB12 → 推挽输出, 初始高电平 (LED 灭) */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    /* LED1: PA8 */
    GPIO_InitStruct.Pin = LED1_PIN;
    HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);
    LED_OFF(LED1_PORT, LED1_PIN);

    /* LED2: PB14 */
    GPIO_InitStruct.Pin = LED2_PIN;
    HAL_GPIO_Init(LED2_PORT, &GPIO_InitStruct);
    LED_OFF(LED2_PORT, LED2_PIN);

    /* LED3: PB12 */
    GPIO_InitStruct.Pin = LED3_PIN;
    HAL_GPIO_Init(LED3_PORT, &GPIO_InitStruct);
    LED_OFF(LED3_PORT, LED3_PIN);
}

/* ====================== ADC1 初始化 ====================== */
/**
  * @brief  ADC1, 通道6 (PA6), 12位, 连续模式, 软件触发
  *         采样时间: 55.5 周期
  *         ADC 时钟 = PCLK2/6 ≈ 11.67MHz (≤14MHz)
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /* --- 使能时钟 --- */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* --- PA6 → 模拟输入 --- */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* --- ADC 基础配置 --- */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;      /* 单通道 */
    hadc1.Init.ContinuousConvMode    = ENABLE;                /* 连续转换 */
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;    /* 软件触发 */
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;   /* 右对齐 */
    hadc1.Init.NbrOfConversion       = 1;                     /* 1 个通道 */
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* --- 通道配置 --- */
    sConfig.Channel      = ADC_CHANNEL_6;          /* ADC1_IN6 (PA6) */
    sConfig.Rank         = ADC_REGULAR_RANK_1;     /* 规则组 Rank1 */
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5; /* 55.5 周期 */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ====================== USART1 初始化 ====================== */
/**
  * @brief  USART1 → PB6(TX) / PB7(RX), 115200-8N1
  *         需要 AFIO 重映射: PA9/PA10 → PB6/PB7
  */
static void MX_USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* --- 使能时钟 --- */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();                    /* AFIO 必须使能! */

    /* --- USART1 重映射到 PB6/PB7 --- */
    __HAL_AFIO_REMAP_USART1_ENABLE();

    /* --- PB6 → USART1_TX (复用推挽输出) --- */
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* --- PB7 → USART1_RX (浮空输入) --- */
    GPIO_InitStruct.Pin       = GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* --- UART 参数配置 --- */
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ====================== 主函数 ====================== */
int main(void)
{
    /* --- HAL 库初始化 --- */
    HAL_Init();

    /* --- 系统时钟: HSE 10MHz → PLL 70MHz --- */
    SystemClock_Config();

    /* --- 外设初始化 --- */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART1_Init();

    /* --- 启动 ADC 连续转换 --- */
    HAL_ADC_Start(&hadc1);

    /* 标志位: LED 当前状态 (0=灭, 1=亮) */
    uint8_t led_state = 0;

    /* ====================== 主循环 ====================== */
    while (1)
    {
        /* 等待 ADC 转换完成 */
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            /* --- 读取 ADC 码值 (12位: 0-4095) --- */
            uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

            /* --- 计算引脚电压 --- */
            float v_adc = (float)adc_value / 4096.0f * 3.3f;

            /* --- 温度计算 --- */
            /* 公式: ((V / 17.47 / 0.625) × 1000 - 100) / 0.385 */
            float temp = ((v_adc / 17.47f / 0.625f) * 1000.0f - 100.0f) / 0.385f;

            /* --- 串口发送 --- */
            char tx_buf[100];
            int len = snprintf(tx_buf, sizeof(tx_buf),
                               "ADC码值：%lu。温度：%.1f。\r\n",
                               adc_value, temp);
            HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 100);
        }

        /* --- LED 交替亮灭 (3个灯同时翻转) --- */
        if (led_state)
        {
            LED_ON(LED1_PORT, LED1_PIN);
            LED_ON(LED2_PORT, LED2_PIN);
            LED_ON(LED3_PORT, LED3_PIN);
        }
        else
        {
            LED_OFF(LED1_PORT, LED1_PIN);
            LED_OFF(LED2_PORT, LED2_PIN);
            LED_OFF(LED3_PORT, LED3_PIN);
        }
        led_state = !led_state;

        /* --- 延时 1 秒 --- */
        HAL_Delay(1000);
    }
}

/* ====================== 错误处理 ====================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* 死循环，等待复位 */
    }
}

/* ====================== assert_failed (HAL 断言用) ====================== */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* 可在此处加断点 */
    while (1) {}
}
#endif
