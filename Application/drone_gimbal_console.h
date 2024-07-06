#ifndef	DRONE_GIMBAL_CONSOLE_H
#define	DRONE_GIMBAL_CONSOLE_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "user_lib.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    PREPARE_MODE = 0,       //初始化
    NORMAL_MODE,            //正常运行模式
    SAFETY_MODE,              //安全模式（停止运动）
} CtrlMode_e;

typedef enum
{
    GIMBAL_RELEASE_CMD = 0,
    GIMBAL_INIT_CMD,
    GIMBAL_NORMAL_CMD,
    GIMBAL_VISION_AIM_CMD,
} Gimbal_CMD_e;

typedef enum
{
    UI_OFF_CMD = 0,
    UI_ON_CMD,
}UI_CMD_e;

typedef enum
{
    SHOOT_RELEASE_CMD = 0,
    SHOOT_START_CMD,
    SHOOT_STOP_CMD,
} Shoot_CMD_e;

typedef enum
{
    STOP_FIRE_CMD = 0, //停止射击
    ONE_FIRE_CMD,      //单发模式
    RAPID_FIRE_CMD,    //无限制连发模式
} ShootFire_CMD_e;

typedef struct
{
    RC_Info_t* rc;
    CtrlMode_e ctrl_mode;
    Gimbal_CMD_e gimbal_cmd;
    Shoot_CMD_e shoot_cmd;

    struct
    {
        fp32 pitch_v;
        fp32 yaw_v;
    } gimbal;

    struct
    {
        ShootFire_CMD_e fire_cmd;
	  		ShootFire_CMD_e last_fire_cmd;
    } shoot;
    
    UI_CMD_e    ui_cmd;
		
} Console_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ConsoleTaskInit(void);
Console_t* Console_Pointer(void);

#endif
