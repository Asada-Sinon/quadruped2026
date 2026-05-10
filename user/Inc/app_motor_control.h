#ifndef APP_MOTOR_CONTROL_H
#define APP_MOTOR_CONTROL_H

#include <stdint.h>

extern volatile uint8_t g_debug_motor_feedback_ok_count;

void AppMotor_SetMainMotorCommandDefaults(float T,
                                          float W,
                                          float K_P,
                                          float K_W,
                                          unsigned short mode,
                                          uint8_t reset_pos);
void AppMotor_PublishCommandSnapshot(void);
void AppMotor_ResetInterpolationState(void);
void AppMotor_SetInterpolationDtMs(float dt_ms);
uint8_t AppMotor_AllFeedbackValid(void);
uint8_t AppMotor_IsKpKwArmed(void);
void AppMotor_SetKpKwArmed(uint8_t armed);
uint8_t AppMotor_GetFeedbackOkCount(void);

#endif
