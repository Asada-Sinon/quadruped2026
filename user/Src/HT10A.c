#include "HT10A.h"
#include "usart.h"

#define HT10A_FRAME_SIZE 30U
#define HT10A_CHANNEL_COUNT 16U
#define HT10A_CHANNEL_BITS 11U
#define HT10A_PAYLOAD_BIT_OFFSET 8U
#define HT10A_CHANNEL_MASK 0x07FFU
#define HT10A_CHANNEL_CENTER 992
#define HT10A_AXIS_SCALE 10
#define HT10A_SWITCH_SCALE 800

remote_control Teaching_Pendant;
uint8_t Teaching_Pendant_buffer[HT10A_FRAME_SIZE];

static int16_t HT10A_DecodeChannel(const uint8_t *buffer, uint8_t channel_idx)
{
    uint16_t bit_offset = (uint16_t)(HT10A_PAYLOAD_BIT_OFFSET + (uint16_t)channel_idx * HT10A_CHANNEL_BITS);
    uint16_t byte_idx = (uint16_t)(bit_offset / 8U);
    uint8_t bit_shift = (uint8_t)(bit_offset % 8U);
    uint32_t raw = (uint32_t)buffer[byte_idx] |
                   ((uint32_t)buffer[byte_idx + 1U] << 8) |
                   ((uint32_t)buffer[byte_idx + 2U] << 16);

    return (int16_t)((raw >> bit_shift) & HT10A_CHANNEL_MASK);
}

static float HT10A_ChannelToAxis(int16_t channel)
{
    return (float)((channel - HT10A_CHANNEL_CENTER) * HT10A_AXIS_SCALE);
}

static float HT10A_ChannelToSwitch(int16_t channel)
{
    return (float)((channel - HT10A_CHANNEL_CENTER) / HT10A_SWITCH_SCALE);
}

void HT10A_process(uint8_t buffer[30])
{
    int16_t channels[HT10A_CHANNEL_COUNT];
    uint8_t i;

    if ((buffer == 0) || (buffer[0] != 0x0F) || (buffer[24] != 0x00))
    {
        return;
    }

    for (i = 0U; i < HT10A_CHANNEL_COUNT; i++)
    {
        channels[i] = HT10A_DecodeChannel(buffer, i);
    }

    Teaching_Pendant.Vy = HT10A_ChannelToAxis(channels[3]);
    Teaching_Pendant.Vx = HT10A_ChannelToAxis(channels[2]);
    Teaching_Pendant.Vw = HT10A_ChannelToAxis(channels[0]);

    Teaching_Pendant.Button1 = HT10A_ChannelToSwitch(channels[4]);
    Teaching_Pendant.Button2 = HT10A_ChannelToSwitch(channels[5]);
    Teaching_Pendant.Button3 = HT10A_ChannelToSwitch(channels[6]);
    Teaching_Pendant.Button4 = HT10A_ChannelToSwitch(channels[7]);

    Teaching_Pendant.switch5 = HT10A_ChannelToAxis(channels[8]);
    Teaching_Pendant.switch6 = HT10A_ChannelToAxis(channels[9]);
}

void Teaching_Pendant_Restart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &Teaching_Pendant_buffer[0], HT10A_FRAME_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}
