#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "IMU/imu_driver.h"
#include "Motor/motor.h"
#include "pid.h"
#include "drone_gimbal_console.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum
{
    GIMBAL_RELAX = 0,          //��ȫģʽ
    GIMBAL_INIT,
    GIMBAL_NORMAL,
    GIMBAL_VISION_AIM,
} GimbalCtrlMode_e;

typedef enum
{
    RAW_VALUE_MODE = 0,
    GYRO_MODE,
    ENCONDE_MODE,
} GimbalMotorMode_e;

typedef struct
{
    fp32            relative_angle; /* unit: degree */
    fp32            gyro_angle;
    fp32            palstance;      /* uint: degree/s */
} GimbalSensor_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    fp32            angle_ref;
    fp32            angle_fdb;
    fp32            speed_ref;
    fp32            speed_fdb;
} Gimbal_PID_t;

typedef struct
{
    MotorInfo_t*    motor_info;
    uint16_t        offset_ecd;
    fp32            ecd_ratio;
    fp32            max_relative_angle;
    fp32            min_relative_angle;

    GimbalMotorMode_e mode;
    GimbalMotorMode_e last_mode;
    fp32            given_value;
	  fp32            last_given_value;
    GimbalSensor_t  sensor;
    Gimbal_PID_t    pid;

    int16_t         current_set;
} GimbalMotor_t;

typedef struct
{
    Console_t*          console;
    IMU_Data_t*         imu;              //
    CAN_Object_t*       gimbal_can;       //
	
    GimbalCtrlMode_e    ctrl_mode;       //
    GimbalCtrlMode_e    last_ctrl_mode;  //
    
    pid_t               gimbal_vision_yaw_pid;
    pid_t               gimbal_vision_pitch_pid;

    GimbalMotor_t       yaw_motor;
    GimbalMotor_t       pitch_motor;
} GimbalHandle_t;

/* �궨�� --------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/
void GimbalAppConfig(void);

#endif
