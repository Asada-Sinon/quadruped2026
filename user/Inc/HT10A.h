#ifndef __TEACHING__PENDANT__
#define __TEACHING__PENDANT__
#include "stdint.h"

typedef struct
{
    float Button1; // 三档带回弹，默认是0,Fire-1
    float Button2; // 两档,自动-1/手动1切换
    float Button3; // 两档，自动-1/手动1切换
    float Button4; // 三档没回弹,默认是0,Reset-1/Route_Death1
    float switch5; // 旋钮左
    float switch6; // 旋钮右
    float Vx; // X方向速度狗头方向
    float Vy; // Y方向速度狗左手方向
    float Vw; // 角速度
} remote_control;
extern remote_control Teaching_Pendant; // 手柄数据结构体
extern uint8_t Teaching_Pendant_buffer[30];
// 函数声明
void Teaching_Pendant_Restart(void);
void HT10A_process(uint8_t buffer[30]);

#endif

