/**
  ******************************************************************************
  * @file    pid.h
  * @brief   增量式 PID 控制器
  *          用于温度控制，输出 0~100% 占空比
  ******************************************************************************
  */

#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* ====================== PID 控制器结构体 ====================== */
typedef struct {
    float Kp;               /* 比例系数 */
    float Ki;               /* 积分系数 */
    float Kd;               /* 微分系数 */
    float setpoint;         /* 目标温度 (℃) */
    float integral;         /* 积分累积项 */
    float prev_error;       /* 上次误差 */
    float output;           /* 当前输出 (0~100%) */
    uint32_t last_tick;     /* 上次计算时的 tick (ms) */
} PID_Controller;

/* ====================== 函数声明 ====================== */

/**
  * @brief  初始化 PID 控制器
  * @param  pid: 控制器指针
  * @param  Kp, Ki, Kd: PID 参数
  * @param  setpoint: 目标温度
  */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint);

/**
  * @brief  执行一次 PID 计算
  * @param  pid: 控制器指针
  * @param  current: 当前温度 (℃)
  * @param  tick_ms: 当前系统节拍 (ms)，用于计算 dt
  * @return 输出值 (0~100)，表示加热占空比
  */
float PID_Compute(PID_Controller *pid, float current, uint32_t tick_ms);

/**
  * @brief  修改 PID 参数
  */
void PID_SetParams(PID_Controller *pid, float Kp, float Ki, float Kd);

/**
  * @brief  修改目标温度
  */
void PID_SetSetpoint(PID_Controller *pid, float setpoint);

/**
  * @brief  重置 PID 状态 (积分清零)
  */
void PID_Reset(PID_Controller *pid);

#endif /* __PID_H */
