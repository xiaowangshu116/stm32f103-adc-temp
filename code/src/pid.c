/**
  ******************************************************************************
  * @file    pid.c
  * @brief   增量式 PID 控制器实现
  *          特性: 积分分离、输出限幅、死区控制
  ******************************************************************************
  */

#include "pid.h"
#include <string.h>

/* PID 输出限幅 */
#define PID_OUTPUT_MAX   100.0f
#define PID_OUTPUT_MIN     0.0f

/* 积分分离阈值: 误差超过此值时不累加积分 */
#define PID_I_SEP_THRESH   5.0f

/* 死区: 误差小于此值时不更新输出 */
#define PID_DEADBAND       0.1f

/* 积分限幅 */
#define PID_I_MAX         50.0f
#define PID_I_MIN        -10.0f

/* ====================== 初始化 ====================== */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint)
{
    memset(pid, 0, sizeof(PID_Controller));
    pid->Kp       = Kp;
    pid->Ki       = Ki;
    pid->Kd       = Kd;
    pid->setpoint = setpoint;
    pid->output   = 0.0f;
    pid->last_tick = 0;
}

/* ====================== PID 计算 ====================== */
float PID_Compute(PID_Controller *pid, float current, uint32_t tick_ms)
{
    /* --- 计算时间间隔 (秒) --- */
    float dt;
    if (pid->last_tick == 0) {
        dt = 0.1f;  /* 首次调用使用默认 100ms */
    } else {
        dt = (float)(tick_ms - pid->last_tick) / 1000.0f;
        if (dt < 0.01f) dt = 0.1f;   /* 防止 dt 过小 */
        if (dt > 1.0f)  dt = 1.0f;   /* 防止 dt 过大 */
    }
    pid->last_tick = tick_ms;

    /* --- 计算误差 --- */
    float error = pid->setpoint - current;

    /* --- 死区: 误差很小时不调整 --- */
    if (error < PID_DEADBAND && error > -PID_DEADBAND) {
        return pid->output;
    }

    /* --- 积分分离: 大偏差时不累加积分 --- */
    if (error < PID_I_SEP_THRESH && error > -PID_I_SEP_THRESH) {
        pid->integral += error * dt;
    } else {
        pid->integral = 0.0f;  /* 偏差大，清零积分 */
    }

    /* 积分限幅 */
    if (pid->integral > PID_I_MAX)  pid->integral = PID_I_MAX;
    if (pid->integral < PID_I_MIN)  pid->integral = PID_I_MIN;

    /* --- 微分 --- */
    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    /* --- PID 输出 --- */
    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * derivative;

    /* 输出限幅 */
    if (output > PID_OUTPUT_MAX) {
        output = PID_OUTPUT_MAX;
        /* 输出饱和时防止积分继续累加 (抗饱和) */
        if (error > 0) pid->integral -= error * dt;  /* 回退积分 */
    }
    if (output < PID_OUTPUT_MIN) {
        output = PID_OUTPUT_MIN;
        if (error < 0) pid->integral -= error * dt;
    }

    pid->output = output;
    return output;
}

/* ====================== 修改参数 ====================== */
void PID_SetParams(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

/* ====================== 修改目标温度 ====================== */
void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/* ====================== 重置 ====================== */
void PID_Reset(PID_Controller *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
    pid->last_tick  = 0;
}
