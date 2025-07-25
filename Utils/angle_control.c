#include "angle_control.h"
#include "pid_control.h"
#include "jy61p.h"
#include <math.h>

void _AnglePID_Reset(void);
void _AnglePID_SetTarget(float target_angle);
void _Angle_Straight_PID_Set(float speed);
void _Angle_Self_PID_Set(void);

/**
 * @brief 初始化角度控制系统
 * @retval None
 */
void AngleControl_Init(void)
{
    /* 初始化PID参数 */
    angle_pid_yaw.Kp = ANGLE_PID_KP;
    angle_pid_yaw.Ki = ANGLE_PID_KI;
    angle_pid_yaw.Kd = ANGLE_PID_KD;

    /* 重置PID状态 */
    angle_pid_yaw.setpoint = 0.0f;
    angle_pid_yaw.feedback = 0.0f;
    angle_pid_yaw.error = 0.0f;
    angle_pid_yaw.last_error = 0.0f;
    angle_pid_yaw.integral = 0.0f;
    angle_pid_yaw.derivative = 0.0f;
    angle_pid_yaw.output = 0.0f;
    angle_pid_yaw.final_output = 0.0f;

    /* 设置输出限制 */
    angle_pid_yaw.output_min = ANGLE_PID_OUTPUT_MIN;
    angle_pid_yaw.output_max = ANGLE_PID_OUTPUT_MAX;

    angle_pid_yaw.angle_mode = PID_MODE_MANUAL;
    angle_pid_yaw.base_speed = 0.0f; // 初始基础速度为0
}

/**
 * @brief 重置角度控制系统（仅Yaw轴）
 * @retval None
 */
void AngleControl_Reset(void)
{
    /* 重置PID状态 */
    _AnglePID_Reset();
    /* 同时更新PID目标值与单纯初始化不同，设置当前角度，防止突然行动 */
    _AnglePID_SetTarget(My_YawAngle);
}

/**
 * @brief 设置直行模式（保持偏航角为0或指定角度）
 * @param speed: 直行速度(RPM)
 * @param target_yaw: 目标偏航角(度)，默认0为直行
 * @retval None
 */
void SetStraightDrive(float speed, float target_yaw)
{
    float rpp_speed = speed * (ENCODER_RESOLUTION * GEAR_RATIO) / 60.0f; // 转换为RPP
    _Angle_Straight_PID_Set(rpp_speed); // 重置PID状态
    /* 角度PID设置目标值 */
    _AnglePID_SetTarget(target_yaw);
}

/**
 * @brief 设置目标偏航角
 * @param target_yaw: 目标Yaw角度(度)
 * @retval None
 */
void SelfTurnTarget(float target_yaw)
{
    _Angle_Self_PID_Set();
    /* 角度PID设置目标值 */
    _AnglePID_SetTarget(target_yaw);
}


/**
 * @brief 初始化角度环PID控制器 私有函数 默认状态时直行
 * @retval None
 */
void _Angle_Self_PID_Set(void)
{
    /* 初始化PID参数 */
    angle_pid_yaw.Kp = ANGLE_SELF_KP;
    angle_pid_yaw.Ki = ANGLE_SELF_KI;
    angle_pid_yaw.Kd = ANGLE_SELF_KD;

    /* 重置PID状态 */
    angle_pid_yaw.setpoint = 0.0f;
    angle_pid_yaw.feedback = 0.0f;
    angle_pid_yaw.error = 0.0f;
    angle_pid_yaw.last_error = 0.0f;
    angle_pid_yaw.integral = 0.0f;
    angle_pid_yaw.derivative = 0.0f;
    angle_pid_yaw.output = 0.0f;
    angle_pid_yaw.final_output = 0.0f;

    /* 设置输出限制 */
    angle_pid_yaw.output_min = ANGLE_SELF_OUTPUT_MIN;
    angle_pid_yaw.output_max = ANGLE_SELF_OUTPUT_MAX;

    angle_pid_yaw.angle_mode = PID_MODE_TURN_IN_PLACE;
    angle_pid_yaw.base_speed = 0.0f; // 初始基础速度为0
}


/**
 * @brief 初始化直行控制
 * @retval None
 */
void _Angle_Straight_PID_Set(float speed)
{
    /* 初始化PID参数 */
    angle_pid_yaw.Kp = ANGLE_PID_KP;
    angle_pid_yaw.Ki = ANGLE_PID_KI;
    angle_pid_yaw.Kd = ANGLE_PID_KD;

    /* 重置PID状态 */
    angle_pid_yaw.setpoint = 0.0f;
    angle_pid_yaw.feedback = 0.0f;
    angle_pid_yaw.error = 0.0f;
    angle_pid_yaw.last_error = 0.0f;
    angle_pid_yaw.integral = 0.0f;
    angle_pid_yaw.derivative = 0.0f;
    angle_pid_yaw.output = 0.0f;
    angle_pid_yaw.final_output = 0.0f;

    /* 设置输出限制 */
    angle_pid_yaw.output_min = ANGLE_PID_OUTPUT_MIN;
    angle_pid_yaw.output_max = ANGLE_PID_OUTPUT_MAX;

    angle_pid_yaw.angle_mode = PID_MODE_STRAIGHT_DRIVE;
    angle_pid_yaw.base_speed = speed; // 初始基础速度为0
}


/**
 * @brief 设置角度PID目标值 - 私有函数
 * @param target_angle: 目标角度(度)
 * @retval None
 */
void _AnglePID_SetTarget(float target_angle)
{
    angle_pid_yaw.setpoint = target_angle;
}

/**
 * @brief 重置角度PID状态 - 私有函数
 * @retval None
 */
void _AnglePID_Reset(void)
{
    angle_pid_yaw.error = 0.0f;
    angle_pid_yaw.last_error = 0.0f;
    angle_pid_yaw.integral = 0.0f;
    angle_pid_yaw.derivative = 0.0f;
    angle_pid_yaw.output = 0.0f;
}


/**
 * @brief 停止所有电机
 * @retval None
 */
void PID_StopAll(void)
{
    /* 停止电机 */
    PID_SetSpeed(PID_MOTOR_A, 0.0f,0);
    PID_SetSpeed(PID_MOTOR_B, 0.0f,0);

    /* 重置控制状态 */
    angle_pid_yaw.angle_mode = PID_MODE_MANUAL;
    angle_pid_yaw.base_speed = 0.0f;

    /* 禁用并重置角度PID */
    _AnglePID_Reset();
}