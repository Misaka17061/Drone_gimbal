#ifndef	SHOOT_TASK_H
#define SHOOT_TASK_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "Motor/motor.h"
#include "Motor/blocked.h"
#include "pid.h"
#include "drone_gimbal_console.h"
#include "user_protocol.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    SHOOT_RELAX = 0,         
    SHOOT_START,
    SHOOT_STOP,
} ShootCtrlMode_e;

typedef enum
{
    TRIGGER_END = 0,
    TRIGGER_BEGIN,
    TRIGGERING
} TriggerState_e;

typedef struct
{
    MotorInfo_t*    motor_info;
    int32_t         offset_ecd;
    fp32            ecd_ratio;

    Double_PID_t    pid;
    fp32            speed;
    fp32            angle;
    fp32            set_speed;
    fp32            set_angle;

    int16_t         current_set;
} ShootMotor_t;

typedef struct
{
    MotorInfo_t*    motor_info;

    pid_t           pid;
    fp32            set_speed;

    int16_t         current_set;
} FrictionWheelMotor_t;

typedef struct
{
    Console_t*      console;
    CAN_Object_t*   shoot_can;    //
	
    ShootCtrlMode_e ctrl_mode;
    ShootMotor_t    trigger_motor;
    FrictionWheelMotor_t  fric_wheel_motor[2];
  
    
    TriggerState_e  trigger_state;
		float shooter_speedfeedback;

    uint16_t        shooter_heat_cooling_rate;
    uint16_t        shooter_heat_cooling_limit;
    uint16_t        shooter_speed_limit;
    uint16_t        shooter_heat;
		uint16_t        shooter_heat2;
	  uint16_t        trigger_last_angle;
    uint16_t        trigger_angle;
    uint16_t        shooter_heat_cooling_rate_k;		
		
} ShootHandle_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ShootTaskInit(void);

#endif
