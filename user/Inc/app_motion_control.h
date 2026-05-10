#ifndef APP_MOTION_CONTROL_H
#define APP_MOTION_CONTROL_H

#include "app_robot.h"

void AppMotion_Init(void);
void AppMotion_UpdateFootTargets(float dt_s);

#endif
