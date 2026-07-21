#ifndef HOST_COMPUTER_UART_PROTOCOL_H
#define HOST_COMPUTER_UART_PROTOCOL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 文本命令单行最大长度，包含字符串结尾的 '\0'。 */
#define PROTOCOL_LINE_MAX_LEN 96

/* 应用层工作模式。 */
typedef enum
{
    PROTOCOL_MODE_MANUAL = 0,
    PROTOCOL_MODE_AUTO,
    PROTOCOL_MODE_AVOID,
    PROTOCOL_MODE_SPEED,
    PROTOCOL_MODE_POSITION
} ProtocolMode;

/* 文本协议和二进制协议共用的应用状态。 */
typedef struct
{
    uint8_t running;          /* 0：停止，1：运行 */
    float target_speed;       /* 目标速度 */
    int32_t target_position;  /* 目标位置 */
    float kp;                 /* PID 比例系数 */
    float ki;                 /* PID 积分系数 */
    float kd;                 /* PID 微分系数 */
    ProtocolMode mode;        /* 当前工作模式 */
    uint32_t start_tick;      /* 本次启动的毫秒时间戳 */
    char last_error[32];      /* 最近一次应用错误文本 */
} ProtocolState;

/** 初始化协议状态并启动 UART 单字节中断接收。 */
void Protocol_Init(UART_HandleTypeDef *huart);

/** 主循环任务：处理接收队列、二进制帧和错误标志。 */
void Protocol_Task(void);

/** 解析并执行一条已经去掉换行符的文本命令。 */
void Protocol_ProcessLine(char *line);

/** 由 HAL_UART_RxCpltCallback 调用，只负责接收和组帧。 */
void Protocol_RxCpltCallback(UART_HandleTypeDef *huart);

/** 由 HAL_UART_ErrorCallback 调用，记录错误并恢复接收。 */
void Protocol_ErrorCallback(UART_HandleTypeDef *huart);

/* 文本协议响应接口。 */
void Protocol_SendOK(void);
void Protocol_SendError(const char *message);
void Protocol_SendStatus(void);
void Protocol_SendSpeed(float left_speed, float right_speed);
void Protocol_SendPosition(int32_t left_pos, int32_t right_pos);
void Protocol_SendSensor(float distance, float voltage, float temperature);
void Protocol_SendPID(void);
void Protocol_SendTarget(void);

const ProtocolState *Protocol_GetState(void);

/*
 * 应用层接口使用 weak 默认实现。
 * 在其他 .c 文件中定义同名函数，即可接入真实电机、PID 和编码器逻辑。
 */
void App_Start(void);
void App_Stop(void);
void App_Reset(void);
void App_SetTargetSpeed(float speed);
void App_SetTargetPosition(int32_t position);
void App_SetPID(float kp, float ki, float kd);
void App_SetMode(ProtocolMode mode);
void App_GetSpeed(float *left_speed, float *right_speed);
void App_GetPosition(int32_t *left_pos, int32_t *right_pos);

#ifdef __cplusplus
}
#endif

#endif
