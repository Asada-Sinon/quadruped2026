#include "vofa.h"
#include <string.h>
#include "usart.h"
/* JustFloat协议固定帧尾
 * 官方要求每一帧后面都要加这个4字节结束标志
 */
extern UART_HandleTypeDef huart6;   // 你的串口6
VOFA_JF_DMA_HandleTypeDef hvofa;   // 这个要定义成全局变量
static const uint8_t g_vofa_jf_tail[4] = {0x00, 0x00, 0x80, 0x7F};
//多此一举但是写上吧，复用的时候方便
void VOFA_JF_DMA_Init(VOFA_JF_DMA_HandleTypeDef *hvofa, UART_HandleTypeDef *huart)
{
    if (hvofa == NULL)
    {
        return;
    }

    hvofa->huart = huart;     // 保存串口句柄
    memset(hvofa->tx_buf, 0, sizeof(hvofa->tx_buf));
}

HAL_StatusTypeDef VOFA_JF_DMA_Send(VOFA_JF_DMA_HandleTypeDef *hvofa,
                                   const float *data,
                                   uint16_t count)
{
    HAL_StatusTypeDef ret;
    uint16_t data_bytes;

    /* 参数检查 */
    if ((hvofa == NULL) || (hvofa->huart == NULL) || (data == NULL) || (count == 0U))
    {
        return HAL_ERROR;
    }

    /* 防止超过缓冲区最大通道数 */
    if (count > VOFA_JF_MAX_CH)
    {
        return HAL_ERROR;
    }

    /* 一个float占4字节 */
    data_bytes = (uint16_t)(count * sizeof(float));

    /* 先拷贝float数据到发送缓冲区
     * 注意：这里拷贝的是float的原始字节，不是转成字符串
     */
    memcpy(hvofa->tx_buf, data, data_bytes);

    /* 在float数据后面拼接4字节帧尾 */
    memcpy(&hvofa->tx_buf[data_bytes], g_vofa_jf_tail, 4U);

    /* 标记为忙，开始DMA发送 */
    ret = HAL_UART_Transmit_DMA(hvofa->huart,
                                hvofa->tx_buf,
                                (uint16_t)(data_bytes + 4U));

    /* 如果启动DMA失败，要把忙标志清掉 */

    return ret;
}
