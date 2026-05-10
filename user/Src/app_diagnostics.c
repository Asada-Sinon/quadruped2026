#include "app_diagnostics.h"

#include "app_motor_control.h"
#include "app_robot.h"
#include "pc_comm_uart10.h"
#include "vofa.h"

#define APP_VOFA_DIAG_CH_COUNT 76U
#define APP_VOFA_SEND_DIV      20U

#if (VOFA_JF_MAX_CH < APP_VOFA_DIAG_CH_COUNT)
#error "VOFA_JF_MAX_CH must be at least APP_VOFA_DIAG_CH_COUNT"
#endif

float ch[VOFA_JF_MAX_CH] = {0};

void App_vofa_Send(void)
{
    static uint8_t s_vofa_div = 0U;
    static float q[J_NUM];
    static float q_des_filtered[J_NUM];
    static float q_des_raw[J_NUM];
    static float qd[J_NUM];
    uint32_t command_age_ms;
    uint8_t i;

    s_vofa_div++;
    if (s_vofa_div < APP_VOFA_SEND_DIV) {
        return;
    }
    s_vofa_div = 0U;

    App_Get_Model_Joint_Angles(q);
    PCComm_GetQDesUrdf(q_des_filtered);
    PCComm_GetLatestQDesUrdf(q_des_raw);
    App_Get_Model_Joint_Velocities(qd);

    for (i = 0U; i < (uint8_t)J_NUM; i++) {
        ch[i] = q[i];
        ch[12U + i] = q_des_filtered[i];
        ch[24U + i] = q_des_raw[i];
        ch[36U + i] = q_des_filtered[i] - q[i];
        ch[48U + i] = qd[i];
    }

    ch[60U] = (float)PCComm_GetEnable();
    ch[61U] = (float)PCComm_IsPolicyControlAllowed();
    ch[62U] = (float)AppMotor_IsKpKwArmed();
    ch[63U] = (float)g_debug_motor_send_cost_ms;
    ch[64U] = (float)PCComm_GetMode();
    ch[65U] = (float)PCComm_GetFault();
    command_age_ms = PCComm_GetLastCommandAgeMs();
    ch[66U] = (command_age_ms == 0xFFFFFFFFU) ? -1.0f : (float)command_age_ms;
    ch[67U] = (float)PCComm_IsCommandFresh();
    ch[68U] = (float)App_GetControlMode();
    ch[69U] = (float)g_debug_motor_feedback_ok_count;
    ch[70U] = (float)g_policy_motor_kp;
    ch[71U] = (float)g_policy_motor_kw;
    ch[72U] = volecity_cmd[0];
    ch[73U] = volecity_cmd[1];
    ch[74U] = volecity_cmd[2];
    ch[75U] = (float)g_debug_motor_send_loop_count;

    VOFA_JF_DMA_Send(&hvofa, ch, (uint16_t)APP_VOFA_DIAG_CH_COUNT);
}
