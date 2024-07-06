#ifndef DRONE_GIMBAL_DEF_H
#define DRONE_GIMBAL_DEF_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Motor/motor.h"

/* ���Ͷ��� ------------------------------------------------------------------*/

/* �궨�� --------------------------------------------------------------------*/
/******************************************************************************
 *                              CANͨѶID����                                                                    *
 ******************************************************************************/
/*---------------�� ��̨���ID ��---------------*/
#define GIMBAL_MOTOR_CONTROL_STD_ID     MOTOR_5TO8_CONTROL_STD_ID
#define GIMBAL_MOTOR_YAW_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     //Yaw��̨���
#define GIMBAL_MOTOR_PITCH_MESSAGE_ID   MOTOR_5_FEEDBACK_ID     //Pitch��̨���
/*---------------�� ������ID ��---------------*/
#define SHOOT_MOTOR_CONTROL_STD_ID      MOTOR_1TO4_CONTROL_STD_ID
#define FRICTION_WHEEL_1_MESSAGE_ID     MOTOR_1_FEEDBACK_ID
#define FRICTION_WHEEL_2_MESSAGE_ID     MOTOR_2_FEEDBACK_ID
#define TRIGGER_MOTOR_MESSAGE_ID        MOTOR_3_FEEDBACK_ID
#define MAGAZINE_MOTOR_MESSAGE_ID       MOTOR_4_FEEDBACK_ID

#define GIMBAL_DATA_FIFO_SIZE   (1024u)

/******************************************************************************
 *                                                              ��е��װ����                                                                      *
 ******************************************************************************/
#define PITCH_REDUCTION_RATIO       (1.0f)  //pitch���ٱ�
#define YAW_REDUCTION_RATIO         (1.0f)  //yaw���ٱ�
#define PITCH_MOTO_POSITIVE_DIR     (1.0f)  //pitch�����װ����
#define YAW_MOTO_POSITIVE_DIR       (1.0f)  //yaw�����װ����


/******************************************************************************
 *                                                                   �ƶ�����                                                                         *
 ******************************************************************************/
/*-----------------�� ң�� ��-----------------*/
#define RC_GIMBAL_MOVE_RATIO_PIT    0.00025f       //pitch�ƶ�����
#define RC_GIMBAL_MOVE_RATIO_YAW    0.00125f         //yaw�ƶ�����   //0.001
/*---------------�� ������ ��---------------*/
#define KB_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X�᷽������ٶ�
#define KB_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y�᷽������ٶ�
#define KB_GIMBAL_MOVE_RATIO_PIT    0.010f       //pitch�ƶ�����
#define KB_GIMBAL_MOVE_RATIO_YAW    0.015f        //yaw�ƶ�����

#define VS_GIMBAL_MOVE_RATIO_YAW    0.0375f         //yaw�ƶ�����
#define VS_GIMBAL_MOVE_RATIO_PIT    0.0375f         //pitch�ƶ�����


/******************************************************************************
 *                                                                   ��������                                                                         *
 ******************************************************************************/
/*---------------�� ͨ������ ��---------------*/
#define START_TASK_PERIOD           100
#define IMU_TASK_PERIOD             5
#define CONSOLE_TASK_PERIOD         5
#define COMM_TASK_PERIOD            1
#define DETECT_TASK_PERIOD          20
/*---------------�� ��̨���� ��---------------*/
#define GIMBAL_TASK_PERIOD          2
#define GIMBAL_UPLOAD_TIMER_PERIOD  15
#define SHOOT_TASK_PERIOD           4

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/

#endif

