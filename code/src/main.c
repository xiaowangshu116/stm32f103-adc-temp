/**
  ******************************************************************************
  * @file    main.c
  * @brief   STM32 PID 温控器 (基于 GD32F103CBT6)
  *
  *          功能:
  *          - PID 自动温控 (默认 25℃, 上位机串口可调)
  *          - 时间比例输出 HEAT_EN (PA7) 驱动加热片
  *          - 3 路共阳极 LED 间隔 1 秒闪烁
  *          - 串口协议: $CMD=value\n 格式, 支持设定/查询/参数修改
  *
  *          硬件:
  *          - 外部晶振: 10MHz → PLL 70MHz
  *          - ADC:  ADC1_IN6 (PA6) — PT100 温度传感器
  *          - LED:  PA8, PB14, PB12 (共阳极, 低电平点亮)
  *          - HEAT: PA7 (推挽输出, 高电平打开加热)
  *          - UART: USART1 → PB6(TX)/PB7(RX), 115200-8N1
  *          - TIM2: 100Hz (10ms) 时间比例输出基准
  ******************************************************************************
  */

#include "stm32f1xx_hal.h"
#include "pid.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>

/* ====================== 外设句柄 ====================== */
ADC_HandleTypeDef   hadc1;
UART_HandleTypeDef  huart1;
TIM_HandleTypeDef   htim2;

/* ====================== PID 控制器 ====================== */
static PID_Controller pid_ctrl;

/* ====================== 系统状态 ====================== */
typedef enum {
    MODE_AUTO = 0,    /* PID 自动控温 */
    MODE_MANUAL,      /* 手动控制加热 */
} SysMode;

static volatile SysMode  sys_mode   = MODE_AUTO;
static volatile uint8_t  heat_state = 0;     /* 当前加热状态 0/1 */
static volatile uint8_t  tpo_counter = 0;    /* 时间比例输出计数 0~99 */
static volatile uint8_t  tpo_duty    = 0;    /* 时间比例占空比 0~100 */

/* UART 接收相关 */
static volatile uint8_t  rx_byte;
static volatile uint8_t  rx_done = 0;

/* 周期性上报计时 */
static uint32_t last_report_tick = 0;

/* ====================== 函数声明 ====================== */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_Init(void);
static void MX_TIM2_Init(void);
static void ProcessCommand(const ProtocolCmd *cmd);
static void SendResponse(const char *resp);
static void SendPeriodicReport(float temp, uint32_t adc_val);
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

/* 加热控制引脚 */
#define HEAT_PORT       GPIOA
#define HEAT_PIN        GPIO_PIN_7

/* ====================== SysTick 中断 ====================== */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ====================== TIM2 中断 (100Hz) ====================== */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/**
  * @brief  TIM2 周期中断回调 — 时间比例输出 PWM
  *         周期 100 个 tick = 1 秒，分辨率 1%
  *         tpo_counter < tpo_duty → HEAT_EN = HIGH
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        tpo_counter++;
        if (tpo_counter >= 100) {
            tpo_counter = 0;
        }

        /* 根据模式 + 占空比 控制 HEAT_EN */
        uint8_t on;
        if (sys_mode == MODE_MANUAL) {
            on = heat_state;  /* 手动模式直接跟随 */
        } else {
            on = (tpo_counter < tpo_duty) ? 1 : 0;
        }

        if (on) {
            HAL_GPIO_WritePin(HEAT_PORT, HEAT_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(HEAT_PORT, HEAT_PIN, GPIO_PIN_RESET);
        }
    }
}

/* ====================== USART1 RX 中断回调 ====================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        rx_done = 1;
    }
}

/* ====================== 系统时钟: HSE 10MHz → PLL 70MHz ====================== */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL14;        /* 5×14 = 70MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;       /* HCLK  = 70MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;         /* PCLK1 = 35MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         /* PCLK2 = 70MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/* ====================== GPIO 初始化 ====================== */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* LED 引脚: PA8, PB14, PB12 → 推挽输出, 初始高电平 (灭) */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = LED1_PIN;
    HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);
    LED_OFF(LED1_PORT, LED1_PIN);

    GPIO_InitStruct.Pin = LED2_PIN;
    HAL_GPIO_Init(LED2_PORT, &GPIO_InitStruct);
    LED_OFF(LED2_PORT, LED2_PIN);

    GPIO_InitStruct.Pin = LED3_PIN;
    HAL_GPIO_Init(LED3_PORT, &GPIO_InitStruct);
    LED_OFF(LED3_PORT, LED3_PIN);

    /* HEAT_EN: PA7 → 推挽输出, 初始低电平 (关加热) */
    GPIO_InitStruct.Pin   = HEAT_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HEAT_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(HEAT_PORT, HEAT_PIN, GPIO_PIN_RESET);
}

/* ====================== ADC1 初始化 ====================== */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_6;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();
}

/* ====================== USART1 初始化 (PB6/PB7, 115200) ====================== */
static void MX_USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART1_ENABLE();

    /* PB6 → TX */
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB7 → RX */
    GPIO_InitStruct.Pin       = GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

/* ====================== TIM2: 100Hz 时间基准 ====================== */
/**
  * @brief  TIM2 配置为 100Hz (10ms 周期)
  *         70MHz / 7000 / 100 = 100Hz
  */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 7000 - 1;   /* 70M / 7000 = 10kHz */
    htim2.Init.Period            = 100 - 1;     /* 10k / 100 = 100Hz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        Error_Handler();

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Base_Start_IT(&htim2);
}

/* ====================== 协议命令处理 ====================== */
static void ProcessCommand(const ProtocolCmd *cmd)
{
    char resp[100];

    switch (cmd->type) {
    case CMD_TEMP_SET:
        PID_SetSetpoint(&pid_ctrl, cmd->params[0]);
        PID_Reset(&pid_ctrl);  /* 切换目标时重置积分 */
        snprintf(resp, sizeof(resp), "$TEMP OK %.1f\r\n", cmd->params[0]);
        SendResponse(resp);
        break;

    case CMD_TEMP_QUERY: {
        /* 读取当前温度 */
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
        float v_adc = (float)adc_val / 4096.0f * 3.3f;
        float temp  = ((v_adc / 17.47f / 0.625f) * 1000.0f - 100.0f) / 0.385f;
        snprintf(resp, sizeof(resp),
                 "$TEMP %.1f  SET=%.1f  ADC=%lu\r\n",
                 temp, pid_ctrl.setpoint, adc_val);
        SendResponse(resp);
        break;
    }

    case CMD_PID_SET:
        PID_SetParams(&pid_ctrl, cmd->params[0], cmd->params[1], cmd->params[2]);
        snprintf(resp, sizeof(resp),
                 "$PID OK Kp=%.2f  Ki=%.3f  Kd=%.2f\r\n",
                 cmd->params[0], cmd->params[1], cmd->params[2]);
        SendResponse(resp);
        break;

    case CMD_PID_QUERY:
        snprintf(resp, sizeof(resp),
                 "$PID Kp=%.2f  Ki=%.3f  Kd=%.2f\r\n",
                 pid_ctrl.Kp, pid_ctrl.Ki, pid_ctrl.Kd);
        SendResponse(resp);
        break;

    case CMD_MODE_AUTO:
        sys_mode = MODE_AUTO;
        PID_Reset(&pid_ctrl);
        SendResponse("$MODE OK AUTO\r\n");
        break;

    case CMD_MODE_MANUAL:
        sys_mode = MODE_MANUAL;
        SendResponse("$MODE OK MANUAL\r\n");
        break;

    case CMD_HEAT_ON:
        if (sys_mode == MODE_MANUAL) {
            heat_state = 1;
            SendResponse("$HEAT OK 1\r\n");
        } else {
            SendResponse("$HEAT ERR: not in MANUAL mode\r\n");
        }
        break;

    case CMD_HEAT_OFF:
        if (sys_mode == MODE_MANUAL) {
            heat_state = 0;
            SendResponse("$HEAT OK 0\r\n");
        } else {
            SendResponse("$HEAT ERR: not in MANUAL mode\r\n");
        }
        break;

    case CMD_STATUS: {
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
        float v_adc = (float)adc_val / 4096.0f * 3.3f;
        float temp  = ((v_adc / 17.47f / 0.625f) * 1000.0f - 100.0f) / 0.385f;
        uint8_t h_on = HAL_GPIO_ReadPin(HEAT_PORT, HEAT_PIN);
        snprintf(resp, sizeof(resp),
                 "$STATUS T=%.1f  SET=%.1f  PWR=%d  H=%d  "
                 "Kp=%.2f  Ki=%.3f  Kd=%.2f  MODE=%s\r\n",
                 temp, pid_ctrl.setpoint, tpo_duty, h_on,
                 pid_ctrl.Kp, pid_ctrl.Ki, pid_ctrl.Kd,
                 (sys_mode == MODE_AUTO) ? "AUTO" : "MANUAL");
        SendResponse(resp);
        break;
    }

    case CMD_RESET:
        SendResponse("$RST OK\r\n");
        HAL_Delay(50);  /* 等待发送完成 */
        NVIC_SystemReset();
        break;

    case CMD_NONE:
    case CMD_UNKNOWN:
    default:
        snprintf(resp, sizeof(resp),
                 "$ERR unknown command: %s\r\n", cmd->raw);
        SendResponse(resp);
        break;
    }
}

/* ====================== 串口发送响应 ====================== */
static void SendResponse(const char *resp)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)resp, strlen(resp), 100);
}

/* ====================== 周期性上报 (每秒) ====================== */
static void SendPeriodicReport(float temp, uint32_t adc_val)
{
    char buf[120];
    uint8_t h_on = HAL_GPIO_ReadPin(HEAT_PORT, HEAT_PIN);

    /* 机器可读格式 */
    int len = snprintf(buf, sizeof(buf),
                       "$RPT ADC=%lu  T=%.1f  SET=%.1f  PWR=%d  H=%d\r\n",
                       adc_val, temp, pid_ctrl.setpoint, tpo_duty, h_on);

    /* 额外发送中文格式 (兼容旧版) */
    len += snprintf(buf + len, sizeof(buf) - len,
                    "ADC码值：%lu。温度：%.1f。\r\n",
                    adc_val, temp);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
}

/* ====================== 主函数 ====================== */
int main(void)
{
    /* --- HAL 初始化 --- */
    HAL_Init();
    SystemClock_Config();

    /* --- 外设初始化 --- */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART1_Init();
    MX_TIM2_Init();

    /* --- PID 控制器初始化 --- */
    PID_Init(&pid_ctrl, 8.0f, 0.1f, 2.0f, 25.0f);

    /* --- 协议解析器初始化 --- */
    Protocol_Init();

    /* --- 启动 ADC 连续转换 --- */
    HAL_ADC_Start(&hadc1);

    /* --- 启动 UART 中断接收 --- */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);

    /* LED 状态标志 */
    uint8_t led_state = 0;

    /* PID 计算计时 */
    uint32_t last_pid_tick = 0;

    /* 开机提示 */
    char boot_msg[] = "$BOOT STM32 PID温控器 v2.0 默认25℃\r\n";
    SendResponse(boot_msg);

    /* ====================== 主循环 ====================== */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* --- 1. 处理串口接收 (每收到一个字符) --- */
        if (rx_done) {
            rx_done = 0;
            if (Protocol_Feed((char)rx_byte)) {
                /* 完整帧收到，解析并执行 */
                ProtocolCmd cmd;
                if (Protocol_Parse(&cmd)) {
                    ProcessCommand(&cmd);
                } else if (cmd.type == CMD_UNKNOWN) {
                    char err[80];
                    snprintf(err, sizeof(err),
                             "$ERR parse error: %s\r\n", cmd.raw);
                    SendResponse(err);
                }
            }
            /* 继续接收下一个字节 */
            HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
        }

        /* --- 2. PID 计算 (每 100ms) --- */
        if (now - last_pid_tick >= 100) {
            last_pid_tick = now;

            /* 读 ADC → 算温度 */
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
                float v_adc = (float)adc_val / 4096.0f * 3.3f;
                float temp  = ((v_adc / 17.47f / 0.625f) * 1000.0f - 100.0f) / 0.385f;

                /* PID 计算 → 更新占空比 */
                if (sys_mode == MODE_AUTO) {
                    float output = PID_Compute(&pid_ctrl, temp, now);
                    tpo_duty = (uint8_t)(output + 0.5f);  /* 四舍五入 */
                    if (tpo_duty > 100) tpo_duty = 100;
                }
            }
        }

        /* --- 3. 每 1 秒: LED 翻转 + 串口上报 --- */
        if (now - last_report_tick >= 1000) {
            last_report_tick = now;

            /* LED 翻转 */
            if (led_state) {
                LED_ON(LED1_PORT, LED1_PIN);
                LED_ON(LED2_PORT, LED2_PIN);
                LED_ON(LED3_PORT, LED3_PIN);
            } else {
                LED_OFF(LED1_PORT, LED1_PIN);
                LED_OFF(LED2_PORT, LED2_PIN);
                LED_OFF(LED3_PORT, LED3_PIN);
            }
            led_state = !led_state;

            /* 串口上报 */
            uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
            float v_adc = (float)adc_val / 4096.0f * 3.3f;
            float temp  = ((v_adc / 17.47f / 0.625f) * 1000.0f - 100.0f) / 0.385f;
            SendPeriodicReport(temp, adc_val);
        }
    }
}

/* ====================== 错误处理 ====================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    while (1) {}
}
#endif
