#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "pid_control.h"

    /* 初级函数声明 */
    void AngleControl_Init(void);
    void AngleControl_Reset(void);
    void PID_StopAll(void);

    /* 功能接口函数声明 */
    void SetStraightDrive(float speed, float target_yaw);
    void SelfTurnTarget(float target_yaw);

    /* 控制模式枚举 */
    typedef enum
    {
        PID_MODE_MANUAL = 0,     // 手动模式
        PID_MODE_STRAIGHT_DRIVE, // 直行保持模式
        PID_MODE_TURN_IN_PLACE,  // 原地转向模式
    } PID_ControlMode;

#ifdef __cplusplus
}
#endif

#endif /* __ANGLE_CONTROL_H */
