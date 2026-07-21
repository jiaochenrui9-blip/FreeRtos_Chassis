#ifndef HOST_COMPUTER_BINARY_PROTOCOL_H
#define HOST_COMPUTER_BINARY_PROTOCOL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 二进制帧格式：AA 55 地址 命令 长度低 长度高 数据... CRC低 CRC高。 */
#define BINARY_PROTOCOL_HEADER_1 0xAAU
#define BINARY_PROTOCOL_HEADER_2 0x55U
#define BINARY_PROTOCOL_DEVICE_ADDRESS 0x01U
#define BINARY_PROTOCOL_BROADCAST_ADDRESS 0xFFU
#define BINARY_PROTOCOL_MAX_PAYLOAD 64U

/* 上位机发给单片机的命令，以及单片机返回的响应类型。 */
typedef enum
{
    BINARY_CMD_START = 0x01,
    BINARY_CMD_STOP = 0x02,
    BINARY_CMD_RESET = 0x03,
    BINARY_CMD_GET_STATUS = 0x04,
    BINARY_CMD_SET_SPEED = 0x10,
    BINARY_CMD_SET_POSITION = 0x11,
    BINARY_CMD_SET_PID = 0x12,
    BINARY_CMD_SET_MODE = 0x13,
    BINARY_CMD_GET_SPEED = 0x20,
    BINARY_CMD_GET_POSITION = 0x21,
    BINARY_CMD_SET_SERVO_POSITION = 0x22,

    BINARY_RSP_ACK = 0x80,
    BINARY_RSP_ERROR = 0x81,
    BINARY_RSP_STATUS = 0x84,
    BINARY_RSP_SPEED = 0xA0,
    BINARY_RSP_POSITION = 0xA1
} BinaryProtocolCommand;

/* 错误响应中的错误码。 */
typedef enum
{
    BINARY_ERROR_NONE = 0x00,
    BINARY_ERROR_BAD_LENGTH = 0x01,
    BINARY_ERROR_BAD_CRC = 0x02,
    BINARY_ERROR_UNKNOWN_COMMAND = 0x03,
    BINARY_ERROR_BAD_VALUE = 0x04,
    BINARY_ERROR_QUEUE_FULL = 0x05
} BinaryProtocolError;

/** 初始化二进制协议，并绑定实际使用的串口。 */
void BinaryProtocol_Init(UART_HandleTypeDef *huart);

/** 在主循环中处理已经接收完整的帧和待发送的错误响应。 */
void BinaryProtocol_Task(void);

/** 丢弃当前未接收完整的二进制帧，重新等待帧头。 */
void BinaryProtocol_ResetReceiver(void);

/**
 * @brief 向二进制状态机送入一个字节。
 * @return 1 表示该字节属于二进制帧；0 表示应交给文本协议处理。
 */
uint8_t BinaryProtocol_InputByte(uint8_t byte);

/** 计算 Modbus 风格 CRC16，初值 0xFFFF，多项式 0xA001。 */
uint16_t BinaryProtocol_CRC16(const uint8_t *data, uint16_t length);

/** 按二进制帧格式发送一条响应。 */
HAL_StatusTypeDef BinaryProtocol_SendFrame(uint8_t command,
                                           const uint8_t *payload,
                                           uint16_t payload_length);

/** 发送左右轮速度，两个 float 均按小端字节序编码。 */
void BinaryProtocol_SendSpeed(float left_speed, float right_speed);

/** 发送左右轮位置，两个 int32_t 均按小端字节序编码。 */
void BinaryProtocol_SendPosition(int32_t left_position, int32_t right_position);

#ifdef __cplusplus
}
#endif

#endif
