/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   串口通信协议解析器实现
  *          帧格式: $CMD[=param1,param2,...]\n
  *          例: $TEMP=25.0\n  $PID=8.0,0.1,2.0\n  $STATUS\n
  ******************************************************************************
  */

#include "protocol.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ====================== 缓冲区 ====================== */
#define RX_BUF_SIZE  128
static char  rx_buf[RX_BUF_SIZE];
static uint8_t rx_idx = 0;

/* ====================== 初始化 ====================== */
void Protocol_Init(void)
{
    memset(rx_buf, 0, sizeof(rx_buf));
    rx_idx = 0;
}

/* ====================== 喂字符 ====================== */
int Protocol_Feed(char c)
{
    /* 忽略回车符 (以 \n 为结束) */
    if (c == '\r') return 0;

    /* 帧结束符 */
    if (c == '\n') {
        rx_buf[rx_idx] = '\0';
        rx_idx = 0;
        return 1;  /* 完整帧就绪 */
    }

    /* 存入缓冲区 (防止溢出) */
    if (rx_idx < RX_BUF_SIZE - 1) {
        rx_buf[rx_idx++] = c;
    }
    return 0;
}

/* ====================== 获取缓冲区 ====================== */
const char* Protocol_GetBuffer(void)
{
    return rx_buf;
}

/* ====================== 解析命令 ====================== */
int Protocol_Parse(ProtocolCmd *cmd)
{
    memset(cmd, 0, sizeof(ProtocolCmd));

    const char *buf = rx_buf;

    /* 必须以 '$' 开头 */
    if (buf[0] != '$') {
        cmd->type = CMD_UNKNOWN;
        return 0;
    }
    buf++;  /* 跳过 '$' */

    /* 记录原始命令 */
    strncpy(cmd->raw, rx_buf, sizeof(cmd->raw) - 1);

    /* ===== 解析命令名 ===== */
    if (strncmp(buf, "TEMP=", 5) == 0) {
        cmd->type = CMD_TEMP_SET;
        buf += 5;
        cmd->params[0] = (float)atof(buf);
        cmd->param_count = 1;
        return 1;
    }
    if (strncmp(buf, "TEMP?", 5) == 0) {
        cmd->type = CMD_TEMP_QUERY;
        return 1;
    }

    if (strncmp(buf, "PID=", 4) == 0) {
        cmd->type = CMD_PID_SET;
        buf += 4;
        /* 解析 Kp,Ki,Kd */
        cmd->params[0] = (float)atof(buf);
        const char *p = strchr(buf, ',');
        if (p) {
            cmd->params[1] = (float)atof(p + 1);
            p = strchr(p + 1, ',');
            if (p) {
                cmd->params[2] = (float)atof(p + 1);
                cmd->param_count = 3;
            } else {
                cmd->param_count = 1;
            }
        } else {
            cmd->param_count = 1;
        }
        return 1;
    }
    if (strncmp(buf, "PID?", 4) == 0) {
        cmd->type = CMD_PID_QUERY;
        return 1;
    }

    if (strncmp(buf, "MODE=AUTO", 9) == 0) {
        cmd->type = CMD_MODE_AUTO;
        return 1;
    }
    if (strncmp(buf, "MODE=MANUAL", 11) == 0) {
        cmd->type = CMD_MODE_MANUAL;
        return 1;
    }

    if (strncmp(buf, "HEAT=1", 6) == 0) {
        cmd->type = CMD_HEAT_ON;
        return 1;
    }
    if (strncmp(buf, "HEAT=0", 6) == 0) {
        cmd->type = CMD_HEAT_OFF;
        return 1;
    }

    if (strncmp(buf, "STATUS", 6) == 0) {
        cmd->type = CMD_STATUS;
        return 1;
    }

    if (strncmp(buf, "RST", 3) == 0) {
        cmd->type = CMD_RESET;
        return 1;
    }

    /* 未知命令 */
    cmd->type = CMD_UNKNOWN;
    return 0;
}
