#include "imu.h"
#include "stdint.h"
#include "usart.h"
#include "string.h"

typedef struct
{
    float alpha;
    float value;
    uint8_t initialized;
} s_LPFilter;

static float lpf_calc_alpha(float sample_dt, float cutoff_hz)
{
    if (sample_dt <= 0.0f || cutoff_hz <= 0.0f)
    {
        return 1.0f;
    }

    const float kLpfPi = 3.14159265358979323846f;
    const float omega = 2.0f * kLpfPi * cutoff_hz;
    const float x = omega * sample_dt;
    float alpha = x / (1.0f + x);

    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    return alpha;
}

static void LPFilter_init(s_LPFilter *filter, float sample_dt, float cutoff_hz)
{
    if (filter == NULL)
    {
        return;
    }

    filter->alpha = lpf_calc_alpha(sample_dt, cutoff_hz);
    filter->value = 0.0f;
    filter->initialized = 0;
}

static void LPFilter(s_LPFilter *filter, float input)
{
    if (filter == NULL)
    {
        return;
    }

    if (!filter->initialized)
    {
        filter->value = input;
        filter->initialized = 1;
        return;
    }

    filter->value += filter->alpha * (input - filter->value);
}

static float LPF_get_value(const s_LPFilter *filter)
{
    if (filter == NULL)
    {
        return 0.0f;
    }

    return filter->value;
}

IMU imu_recv;
IMU_Body imu_body_recv;
/* 接收数据并使用0x91数据包结构定义来解释数据 */
__align(4) id0x91_t dat; /* struct must be 4 byte aligned */
float imu_angle[3];      /* eular angles:R/P/Y */
s_LPFilter lpf_w;
uint8_t lpf_init_flag = 0;
float lpf_weight = 5;
#define IMU_FRAME_HEADER_SIZE 6U
#define IMU_FRAME_PAYLOAD_SIZE ((uint16_t)sizeof(id0x91_t))
#define IMU_FRAME_SIZE (IMU_FRAME_HEADER_SIZE + IMU_FRAME_PAYLOAD_SIZE)
uint8_t imu_rx_buf[IMU_FRAME_SIZE];
static float ABS(float a)
{
    return a > 0 ? a : -a;
}

static void imu_map_to_body(const IMU *src, IMU_Body *dst)
{
    if (src == NULL || dst == NULL)
    {
        return;
    }

    /* 坐标变换：IMU X+ -> 机身 Y-，IMU Y+ -> 机身 X+，IMU Z+ -> 机身 Z+。 */
    dst->acc[0] = src->acc[1];
    dst->acc[1] = -src->acc[0];
    dst->acc[2] = src->acc[2];

    dst->gyro[0] = src->gyro[1];
    dst->gyro[1] = -src->gyro[0];
    dst->gyro[2] = src->gyro[2];

    dst->angle[0] = src->angle[0];  // body roll
    dst->angle[1] = -src->angle[1]; // body pitch
    dst->angle[2] = src->angle[2];  // body yaw
}

// crc校验
static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j = 0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currect_crc = crc;
}

void imu_data_process(uint8_t *receive)
{
    // 初始化低通滤波器
    if (!lpf_init_flag)
    {
        LPFilter_init(&lpf_w, 0.01, lpf_weight);
        lpf_init_flag = 1;
    }
    // 调试用
    //    lpf_w.weight = 1.0f / ( 1.0f + 1.0f/(2.0f * PI * 0.01 * lpf_weight) );
    uint16_t CRCReceived = 0;   /* CRC value received from a frame */
    uint16_t CRCCalculated = 0; /* CRC value caluated from a frame */
    uint8_t payload_len = 0;
    static float imu_Z_last = 0; // 上一时刻z角度,用于累计出z轴旋转的总角度
    float imu_Z_now = 0;
    float imu_Z_temp_1 = 0;
    float imu_Z_temp_2 = 0;
    float imu_Z_temp = 0;
    if (receive[0] == 0x5A && receive[1] == 0xA5) // 帧头
    {
        /* CRC */
        CRCReceived = receive[4] + (receive[5] << 8);
        payload_len = receive[2] + (receive[3] << 8);
        /* calculate CRC */
        crc16_update(&CRCCalculated, receive, 4);
        crc16_update(&CRCCalculated, receive + 6, payload_len);
        /* CRC match */
        if (CRCCalculated == CRCReceived)
        {
            memcpy(&dat, &receive[6], sizeof(id0x91_t));
            /* 计算Z轴累加旋转角度 */
            imu_Z_now = dat.eul[2];
            if (imu_Z_last <= imu_Z_now)
            {
                imu_Z_temp_1 = imu_Z_now - imu_Z_last;
                imu_Z_temp_2 = imu_Z_now - imu_Z_last - 360;
            }
            else
            {
                imu_Z_temp_1 = imu_Z_now - imu_Z_last;
                imu_Z_temp_2 = imu_Z_now - imu_Z_last + 360;
            }
            imu_Z_temp = (ABS(imu_Z_temp_1)) < (ABS(imu_Z_temp_2)) ? imu_Z_temp_1 : imu_Z_temp_2;
            imu_Z_last = imu_Z_now;

            imu_angle[0] = dat.eul[0];
            imu_angle[1] = dat.eul[1];
            imu_angle[2] = imu_angle[2] + imu_Z_temp;

            imu_recv.acc[0] = dat.acc[0];
            imu_recv.acc[1] = dat.acc[1];
            imu_recv.acc[2] = dat.acc[2];

            LPFilter(&lpf_w, dat.gyr[2]);
            imu_recv.gyro[0] = dat.gyr[0];
            imu_recv.gyro[1] = dat.gyr[1];
            imu_recv.gyro[2] = LPF_get_value(&lpf_w);
            //            set_debug_data(2, imu_recv.gyro[2]);
            imu_recv.w_original = dat.gyr[2];
            //            set_debug_data(3, imu_recv.w_original);

            imu_recv.angle[0] = imu_angle[0];
            imu_recv.angle[1] = imu_angle[1];
            imu_recv.angle[2] = imu_angle[2];

            imu_recv.quaternion[0] = dat.quat[0];
            imu_recv.quaternion[1] = dat.quat[1];
            imu_recv.quaternion[2] = dat.quat[2];
            imu_recv.quaternion[3] = dat.quat[3];

            imu_map_to_body(&imu_recv, &imu_body_recv);
        }
    }
}

void IMU_Restart(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart8, imu_rx_buf, sizeof(imu_rx_buf)) != HAL_OK)
    {
        return;
    }

    if (huart8.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    }
}

void IMU_RxEvent(uint16_t size)
{
    if (size >= IMU_FRAME_SIZE)
    {
        imu_data_process(imu_rx_buf);
    }

    IMU_Restart();
}

IMU *imu_get_data(void)
{
    return &imu_recv;
}

IMU_Body *imu_get_body_data(void)
{
    return &imu_body_recv;
}
