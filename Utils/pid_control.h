#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "encoder.h"
#include "motor.h"

    /* PID控制器结构体 */
    typedef struct
    {
        float Kp; // 比例系数
        float Ki; // 积分系数
        float Kd; // 微分系数

        float setpoint;   // 设定值（目标值）
        float feedback;   // 反馈值（当前值）
        float error;      // 当前误差
        float last_error; // 上次误差
        float prev_error; // 上上次误差
        float integral;   // 积分项
        float derivative; // 微分项

        float output;     // PID输出
        float output_min; // 输出下限
        float output_max; // 输出上限

        uint8_t direction; // 电机方向
    } PID_TypeDef;

/* PID控制器参数（可调整） */
// 为电机A和B分别定义PID参数
#define PID_KP_A 0.4f  // 电机A比例系数默认值
#define PID_KI_A 0.1f // 电机A积分系数默认值
#define PID_KD_A 0.3f  // 电机A微分系数默认值

#define PID_KP_B 0.4f          // 电机B比例系数默认值  纯速度为2.0
#define PID_KI_B 0.1f          // 电机B积分系数默认值   1.4
#define PID_KD_B 0.3f          // 电机B微分系数默认值   1.0
#define PID_OUTPUT_MAX 999.0f  // 输出最大值（对应PWM最大值）
#define PID_OUTPUT_MIN -999.0f    // 输出最小值

/* 角度环PID控制器参数 */
#define ANGLE_PID_KP 0.5f       // 角度环比例系数默认值
#define ANGLE_PID_KI 0.05f       // 角度环积分系数默认值  
#define ANGLE_PID_KD 0.2f       // 角度环微分系数默认值
#define ANGLE_SELF_KP 1.2f       // 角度环比例系数默认值
#define ANGLE_SELF_KI 0.1f       // 角度环积分系数默认值  
#define ANGLE_SELF_KD 5.0f       // 角度环微分系数默认值
#define DEAD_ZONE 3.0f // 角度环死区范围(度)
#define ANGLE_PID_OUTPUT_MAX 15.0f // 角度环输出最大值(RPP)
#define ANGLE_PID_OUTPUT_MIN -15.0f // 角度环输出最小值(RPP)
#define ANGLE_SELF_OUTPUT_MAX 400.0f // 角度环输出最大值(RPP)
#define ANGLE_SELF_OUTPUT_MIN -400.0f // 角度环输出最小值(RPP)

/* 定义角度来源(二选一) */
// #define My_YawAngle MPU6050.Yaw_Custom
#define My_YawAngle Yaw


/* 角度PID控制器专用结构体 */
typedef struct
{
    float Kp; // 比例系数
    float Ki; // 积分系数
    float Kd; // 微分系数

    float setpoint;   // 设定值（目标角度）
    float feedback;   // 反馈值（当前角度）
    float error;      // 当前误差
    float last_error; // 上次误差
    float integral;   // 积分项
    float derivative; // 微分项

    float output;     // PID输出(RPP)
    float output_min; // 输出下限
    float output_max; // 输出上限
    float final_output; // 最终输出
    
    float base_speed; // 基础速度
    uint8_t angle_mode; // 角度控制模式


} AnglePID_TypeDef;

    /* 电机PID控制器枚举 */
    typedef enum
    {
        PID_MOTOR_A = 1,
        PID_MOTOR_B = 2
    } PID_Motor_ID;
    
    /* 函数声明 */
    void PID_Init(PID_TypeDef *pid);
    float PID_Compute(PID_TypeDef *pid);
    void PID_SetSpeed(PID_Motor_ID motor_id, float target_rpm, uint8_t is_direct_rpp);
    void PID_Update(void);
    
    /* 角度环PID控制函数声明 */
    void AnglePID_Update(void);
    float AnglePID_Compute(void);
    
    /* 外部变量声明 */
    extern PID_TypeDef pid_motor_a;
    extern PID_TypeDef pid_motor_b;
    extern AnglePID_TypeDef angle_pid_yaw;  // Yaw轴角度PID控制器

    // 声明各自的全局调节参数
    extern float pid_kp_a;
    extern float pid_ki_a;
    extern float pid_kd_a;

    extern float pid_kp_b;
    extern float pid_ki_b;
    extern float pid_kd_b;


#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROL_H */