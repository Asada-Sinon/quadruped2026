#include "HT10A.h"
#include "usart.h"

remote_control Teaching_Pendant;
uint8_t Teaching_Pendant_buffer[30];
void HT10A_process(uint8_t buffer[30])
{
    int16_t _channels[16];
    if (buffer[0] != 0x0F || buffer[24] != 0x00)
        return; // 数据包头尾正确则开始处理
    _channels[0] = (buffer[1] | ((buffer[2] & 0x07) << 8)) & 0x07FF;
    // 通道1: bits 11-21 (byte2低5位 + byte3高6位)
    _channels[1] = ((buffer[2] >> 3) | (buffer[3] << 5)) & 0x07FF;
    // 通道2: bits 22-32 (byte3低2位 + byte4全8位 + byte5高1位)
    _channels[2] = ((buffer[3] >> 6) | (buffer[4] << 2) | ((buffer[5] & 0x01) << 10)) & 0x07FF;
    // 通道3: bits 33-43 (byte5低7位 + byte6高4位)
    _channels[3] = ((buffer[5] >> 1) | (buffer[6] << 7)) & 0x07FF;
    // 通道4: bits 44-54 (byte6低4位 + byte7高7位)
    _channels[4] = ((buffer[6] >> 4) | (buffer[7] << 4)) & 0x07FF;
    // 通道5: bits 55-65 (byte7低1位 + byte8全8位 + byte9高2位)
    _channels[5] = ((buffer[7] >> 7) | (buffer[8] << 1) | ((buffer[9] & 0x03) << 9)) & 0x07FF;
    // 通道6: bits 66-76 (byte9低6位 + byte10高5位)
    _channels[6] = ((buffer[9] >> 2) | (buffer[10] << 6)) & 0x07FF;
    // 通道7: bits 77-87 (byte10低3位 + byte11全8位)
    _channels[7] = ((buffer[10] >> 5) | (buffer[11] << 3)) & 0x07FF;
    // 通道8: bits 88-98 (byte12低8位 + byte13高3位)
    _channels[8] = (buffer[12] | ((buffer[13] & 0x07) << 8)) & 0x07FF;
    // 通道9: bits 99-109 (byte13低5位 + byte14高6位)
    _channels[9] = ((buffer[13] >> 3) | (buffer[14] << 5)) & 0x07FF;
    // 通道10: bits 110-120 (byte14低2位 + byte15全8位 + byte16高1位)
    _channels[10] = ((buffer[14] >> 6) | (buffer[15] << 2) | ((buffer[16] & 0x01) << 10)) & 0x07FF;
    // 通道11: bits 121-131 (byte16低7位 + byte17高4位)
    _channels[11] = ((buffer[16] >> 1) | (buffer[17] << 7)) & 0x07FF;
    // 通道12: bits 132-142 (byte17低4位 + byte18高7位)
    _channels[12] = ((buffer[17] >> 4) | (buffer[18] << 4)) & 0x07FF;
    // 通道13: bits 143-153 (byte18低1位 + byte19全8位 + byte20高2位)
    _channels[13] = ((buffer[18] >> 7) | (buffer[19] << 1) | ((buffer[20] & 0x03) << 9)) & 0x07FF;
    // 通道14: bits 154-164 (byte20低6位 + byte21高5位)
    _channels[14] = ((buffer[20] >> 2) | (buffer[21] << 6)) & 0x07FF;
    // 通道15: bits 165-175 (byte21低3位 + byte22全8位)
    _channels[15] = ((buffer[21] >> 5) | (buffer[22] << 3)) & 0x07FF;

    // 摇杆[-8000,8000]
    // Teaching_Pendant_Data.x = (_channels[3] - 992) * 10;
    // Teaching_Pendant_Data.y = (_channels[2] - 992) * 10;
    // Teaching_Pendant_Data.z = (_channels[0] - 992) * 10;
    Teaching_Pendant.Vy = (_channels[3] - 992) * 10; // Y方向速度
    Teaching_Pendant.Vx = (_channels[2] - 992) * 10; // X方向速度
    Teaching_Pendant.Vw = (_channels[0] - 992) * 10; // 角速度
    // 四个开关
    Teaching_Pendant.Button1 = (_channels[4] - 992) / 800;
    Teaching_Pendant.Button2 = (_channels[5] - 992) / 800;
    Teaching_Pendant.Button3 = (_channels[6] - 992) / 800;
    Teaching_Pendant.Button4 = (_channels[7] - 992) / 800;
    // 旋钮[-8000,8000]
    Teaching_Pendant.switch5 = (_channels[8] - 992) * 10;
    Teaching_Pendant.switch6 = (_channels[9] - 992) * 10;
}

void Teaching_Pendant_Restart(void){
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &Teaching_Pendant_buffer[0], 30);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

