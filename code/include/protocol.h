/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   串口通信协议解析器
  *          格式: $<CMD>[=<参数>]\n
  ******************************************************************************
  */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/* ====================== 命令类型枚举 ====================== */
typedef enum {
    CMD_NONE = 0,
    CMD_TEMP_SET,       /* $TEMP=25.0      - 设定目标温度 */
    CMD_TEMP_QUERY,     /* $TEMP?           - 查询温度 */
    CMD_PID_SET,        /* $PID=8.0,0.1,2.0 - 设定 PID */
    CMD_PID_QUERY,      /* $PID?            - 查询 PID */
    CMD_MODE_AUTO,      /* $MODE=AUTO       - 自动模式 */
    CMD_MODE_MANUAL,    /* $MODE=MANUAL     - 手动模式 */
    CMD_HEAT_ON,        /* $HEAT=1          - 手动开加热 */
    CMD_HEAT_OFF,       /* $HEAT=0          - 手动关加热 */
    CMD_STATUS,         /* $STATUS          - 查询状态 */
    CMD_RESET,          /* $RST             - 软件复位 */
    CMD_UNKNOWN,        /* 未知命令 */
} CmdType;

/* ====================== 解析后的命令结构体 ====================== */
typedef struct {
    CmdType type;
    float   params[3];      /* 最多 3 个浮点参数 */
    uint8_t param_count;    /* 参数个数 */
    char    raw[64];        /* 原始命令字符串 */
} ProtocolCmd;

/* ====================== 函数声明 ====================== */

/**
  * @brief  初始化协议解析器
  */
void Protocol_Init(void);

/**
  * @brief  喂入一个字符
  * @param  c: 接收到的字符
  * @return 1 = 已收到完整一帧 (\n 结束), 0 = 帧未完成
  */
int  Protocol_Feed(char c);

/**
  * @brief  解析收到的帧
  * @param  cmd: 输出解析后的命令结构体
  * @return 1 = 解析成功, 0 = 格式错误
  */
int  Protocol_Parse(ProtocolCmd *cmd);

/**
  * @brief  获取接收缓冲区 (用于直接访问)
  */
const char* Protocol_GetBuffer(void);

#endif /* __PROTOCOL_H */
