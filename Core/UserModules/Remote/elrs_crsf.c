#include "elrs_crsf.h"
#include "main.h"

#include <string.h>

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8u
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16u

#define CRSF_MAX_FRAME_SIZE 64u
#define CRSF_MIN_LENGTH 2u
#define CRSF_MAX_LENGTH 62u
#define CRSF_RC_PAYLOAD_SIZE 22u
#define CRSF_CRC_POLY 0xD5u

typedef enum
{
    CRSF_STATE_WAIT_ADDRESS = 0,
    CRSF_STATE_WAIT_LENGTH,
    CRSF_STATE_READ_FRAME
} CRSF_ParseState;

static volatile uint16_t elrs_channels[ELRS_CRSF_CHANNEL_COUNT];   //通道大小
static volatile uint32_t elrs_last_update_ms;   // 最近更新通道的时间
static volatile uint32_t crsf_valid_frame_count;   //
static volatile uint32_t crsf_crc_error_count;    //
static uint8_t crsf_frame[CRSF_MAX_FRAME_SIZE];   //数据帧数组
static uint8_t crsf_len;   //   crsf数据的长度
static uint8_t crsf_pos;   //下标
static CRSF_ParseState crsf_state;   // crsf状态机
//计算出Crc位，以便和帧包里的crc进行比较
static uint8_t CRSF_CalcCrc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;

    while (len-- != 0u)
    {
        crc ^= *data++;
        for (uint8_t i = 0u; i < 8u; i++)
        {
            if ((crc & 0x80u) != 0u)
            {
                crc = (uint8_t)((crc << 1u) ^ CRSF_CRC_POLY);
            }
            else
            {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }
    return crc;
}
//通过特定的算法计算出某个通道的值大小
static uint16_t CRSF_GetBits11(const uint8_t *payload, uint8_t channel)
{
    uint16_t bit_index = (uint16_t)channel * 11u;
    uint8_t byte_index = (uint8_t)(bit_index >> 3u);
    uint8_t bit_offset = (uint8_t)(bit_index & 0x07u);
    uint32_t value = payload[byte_index];

    if ((byte_index + 1u) < CRSF_RC_PAYLOAD_SIZE)
    {
        value |= (uint32_t)payload[byte_index + 1u] << 8u;
    }
    if ((byte_index + 2u) < CRSF_RC_PAYLOAD_SIZE)
    {
        value |= (uint32_t)payload[byte_index + 2u] << 16u;
    }

    return (uint16_t)((value >> bit_offset) & 0x07FFu);
}
//确认帧数据合法之后，开始逐个解析各个通道的值大小
static void CRSF_ParseChannels(const uint8_t *payload)
{
    uint16_t parsed[ELRS_CRSF_CHANNEL_COUNT];
    uint32_t primask;
//先在外面单独定义一个数组来临时存放通道值
    for (uint8_t i = 0u; i < ELRS_CRSF_CHANNEL_COUNT; i++)
    {
        parsed[i] = CRSF_GetBits11(payload, i);
    }
//关闭中断，防止更新被打扰
    primask = __get_PRIMASK();
    __disable_irq();
    //一口气逐个更新通道值
    for (uint8_t i = 0u; i < ELRS_CRSF_CHANNEL_COUNT; i++)
    {
        elrs_channels[i] = parsed[i];
    }
    //更新上次更新通道的时间
    elrs_last_update_ms = HAL_GetTick();
    if (primask == 0u)
    {
        __enable_irq();
    }
}

/**
 * @brief 处理帧
 *
 *
 */
static void CRSF_ProcessFrame(void)
{
    uint8_t type;
    uint8_t payload_len;
    uint8_t crc;
    uint8_t calc_crc;
//处理帧长度是否合法
    if (crsf_len < CRSF_MIN_LENGTH)
    {
        return;
    }
//更新数据类型，通道数据，校验位
    type = crsf_frame[2];
    payload_len = (uint8_t)(crsf_len - 2u);
    crc = crsf_frame[crsf_len + 1u];
    calc_crc = CRSF_CalcCrc(&crsf_frame[2], (uint8_t)(crsf_len - 1u));
//计算校验位是否和数据发过来的校验位相等
    if (crc != calc_crc)
    {
        crsf_crc_error_count++;
        return;
    }
//判断类型是否为遥控类型，通道长度是否正确
    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && payload_len == CRSF_RC_PAYLOAD_SIZE)
    {
        CRSF_ParseChannels(&crsf_frame[3]);
        crsf_valid_frame_count++;
    }
}
//初始化ELRS数据
void ELRS_CRSF_Init(void)
{
    crsf_state = CRSF_STATE_WAIT_ADDRESS;
    crsf_len = 0u;
    crsf_pos = 0u;
    memset(crsf_frame, 0, sizeof(crsf_frame));
//初始化通道数组的值
    for (uint8_t i = 0u; i < ELRS_CRSF_CHANNEL_COUNT; i++)
    {
        elrs_channels[i] = ELRS_CRSF_VALUE_MID;
    }
    elrs_last_update_ms = 0u;
    crsf_valid_frame_count = 0u;
    crsf_crc_error_count = 0u;
}

/**
 * @brief ELRS回调
 *
 * @param byte 每次接收到的字节
 */
void ELRS_CRSF_UART_RxCallback(uint8_t byte)
{
    //根据状态机的值来进行操作
    switch (crsf_state)
    {
    case CRSF_STATE_WAIT_ADDRESS:
        if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            //更新地址
            crsf_frame[0] = byte;
            crsf_state = CRSF_STATE_WAIT_LENGTH;
        }
        break;

    case CRSF_STATE_WAIT_LENGTH:
        if (byte >= CRSF_MIN_LENGTH && byte <= CRSF_MAX_LENGTH)
        {
            //更新数据长度
            crsf_len = byte;
            crsf_pos = 0u;
            crsf_frame[1] = byte;
            crsf_state = CRSF_STATE_READ_FRAME;
        }
        else
        {
            //数据不合法，重新来
            crsf_state = CRSF_STATE_WAIT_ADDRESS;
        }
        break;

    case CRSF_STATE_READ_FRAME:
        //不断读取数据
        crsf_frame[2u + crsf_pos] = byte;
        crsf_pos++;
        if (crsf_pos >= crsf_len)
        {
            CRSF_ProcessFrame();
            crsf_state = CRSF_STATE_WAIT_ADDRESS;
        }
        break;

    default:
        crsf_state = CRSF_STATE_WAIT_ADDRESS;
        break;
    }
}

/**
 *@brief 判断设备是否在线，因为要求设备一直连接
 *
 * @return 1设备在线online 0设备离线offline
 */
uint8_t ELRS_CRSF_IsOnline(void)
{
    uint32_t last = elrs_last_update_ms;

    if (last == 0u)
    {
        return 0u;
    }
//间隔时间不得超过50ms
    return ((HAL_GetTick() - last) <= ELRS_CRSF_ONLINE_TIMEOUT_MS) ? 1u : 0u;
}
/**
 * @brief 获取更新后ELRS的某一通道的值大小
 * @param ch 指定的通道序号
 * @return 特定的通道值
 */
uint16_t ELRS_CRSF_GetChannel(uint8_t ch)
{
    uint16_t value;
    uint32_t primask;
//判断是否合法
    if (ch < 1u || ch > ELRS_CRSF_CHANNEL_COUNT)
    {
        return 0u;
    }
//关闭串口中断，读取不可被打断
    primask = __get_PRIMASK();
    __disable_irq();
    value = elrs_channels[ch - 1u];
    //开启中断
    if (primask == 0u)
    {
        __enable_irq();
    }

    return value;
}

/**
 * @brief 获取多个通道的值大小
 * @param ch 将通道数组传进去
 */
void ELRS_CRSF_GetChannels(uint16_t ch[ELRS_CRSF_CHANNEL_COUNT])
{
    uint32_t primask;

    if (ch == NULL)
    {
        return;
    }
    //关闭串口中断，读取不可被打断
    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0u; i < ELRS_CRSF_CHANNEL_COUNT; i++)
    {
        ch[i] = elrs_channels[i];
    }
    //开启中断
    if (primask == 0u)
    {
        __enable_irq();
    }
}
//获取上次更新的时间
uint32_t ELRS_CRSF_GetLastUpdateTime(void)
{
    return elrs_last_update_ms;
}
//当前已经读取了多少个合法帧
uint32_t ELRS_CRSF_GetValidFrameCount(void)
{
    return crsf_valid_frame_count;
}
//当前已经读取了多少个错误帧
uint32_t ELRS_CRSF_GetCrcErrorCount(void)
{
    return crsf_crc_error_count;
}
//将原始通道值转变为好看的值 （1000~2000）
/**
 *
 * @param raw 通道的原始数据
 * @return 转换之后的值
 */
uint16_t ELRS_CRSF_RawToUs(uint16_t raw)
{
    if (raw <= ELRS_CRSF_VALUE_MIN)
    {
        return 1000u;
    }
    if (raw >= ELRS_CRSF_VALUE_MAX)
    {
        return 2000u;
    }

    return (uint16_t)(1000u + (((uint32_t)(raw - ELRS_CRSF_VALUE_MIN) * 1000u) /
                              (ELRS_CRSF_VALUE_MAX - ELRS_CRSF_VALUE_MIN)));
}
//将原始通道值转变为更好看的值 （-1000~1000）
/**
 *
 * @param raw 原始值
 * @return 转换之后的大小
 */
int16_t ELRS_CRSF_RawToSigned(uint16_t raw)
{
    if (raw <= ELRS_CRSF_VALUE_MIN)
    {
        return -1000;
    }
    if (raw >= ELRS_CRSF_VALUE_MAX)
    {
        return 1000;
    }

    return (int16_t)(-1000 + (int32_t)(((uint32_t)(raw - ELRS_CRSF_VALUE_MIN) * 2000u) /
                                      (ELRS_CRSF_VALUE_MAX - ELRS_CRSF_VALUE_MIN)));
}
