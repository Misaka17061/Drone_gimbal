#ifndef DRONE_GIMBAL_DEF_H
#define DRONE_GIMBAL_DEF_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "Motor/motor.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/
/******************************************************************************
 *                              CAN通讯ID配置                                                                    *
 ******************************************************************************/
/*---------------↓ 云台电机ID ↓---------------*/
#define GIMBAL_MOTOR_CONTROL_STD_ID     MOTOR_5TO8_CONTROL_STD_ID
#define GIMBAL_MOTOR_YAW_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     //Yaw云台电机
#define GIMBAL_MOTOR_PITCH_MESSAGE_ID   MOTOR_5_FEEDBACK_ID     //Pitch云台电机
/*---------------↓ 射击电机ID ↓---------------*/
#define SHOOT_MOTOR_CONTROL_STD_ID      MOTOR_1TO4_CONTROL_STD_ID
#define FRICTION_WHEEL_1_MESSAGE_ID     MOTOR_1_FEEDBACK_ID
#define FRICTION_WHEEL_2_MESSAGE_ID     MOTOR_2_FEEDBACK_ID
#define TRIGGER_MOTOR_MESSAGE_ID        MOTOR_3_FEEDBACK_ID
#define MAGAZINE_MOTOR_MESSAGE_ID       MOTOR_4_FEEDBACK_ID

#define GIMBAL_DATA_FIFO_SIZE   (1024u)

/******************************************************************************
 *                                                              机械安装参数                                                                      *
 ******************************************************************************/
#define PITCH_REDUCTION_RATIO       (1.0f)  //pitch减速比
#define YAW_REDUCTION_RATIO         (1.0f)  //yaw减速比
#define PITCH_MOTO_POSITIVE_DIR     (1.0f)  //pitch电机安装方向
#define YAW_MOTO_POSITIVE_DIR       (1.0f)  //yaw电机安装方向


/******************************************************************************
 *                                                                   移动控制                                                                         *
 ******************************************************************************/
/*-----------------↓ 遥控 ↓-----------------*/
#define RC_GIMBAL_MOVE_RATIO_PIT    0.00025f       //pitch移动比例
#define RC_GIMBAL_MOVE_RATIO_YAW    0.00125f         //yaw移动比例   //0.001
/*---------------↓ 鼠标键盘 ↓---------------*/
#define KB_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y轴方向最大速度
#define KB_GIMBAL_MOVE_RATIO_PIT    0.010f       //pitch移动比例
#define KB_GIMBAL_MOVE_RATIO_YAW    0.015f        //yaw移动比例

#define VS_GIMBAL_MOVE_RATIO_YAW    0.0375f         //yaw移动比例
#define VS_GIMBAL_MOVE_RATIO_PIT    0.0375f         //pitch移动比例


/******************************************************************************
 *                                                                   任务周期                                                                         *
 ******************************************************************************/
/*---------------↓ 通用任务 ↓---------------*/
#define START_TASK_PERIOD           100
#define IMU_TASK_PERIOD             5
#define CONSOLE_TASK_PERIOD         5
#define COMM_TASK_PERIOD            1
#define DETECT_TASK_PERIOD          20
/*---------------↓ 云台任务 ↓---------------*/
#define GIMBAL_TASK_PERIOD          2
#define GIMBAL_UPLOAD_TIMER_PERIOD  15
#define SHOOT_TASK_PERIOD           4

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/

#endif

