#ifndef	DRONE_GIMBAL_CONSOLE_H
#define	DRONE_GIMBAL_CONSOLE_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "user_lib.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum
{
    PREPARE_MODE = 0,       //��ʼ��
    NORMAL_MODE,            //��������ģʽ
    SAFETY_MODE,              //��ȫģʽ��ֹͣ�˶���
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
    STOP_FIRE_CMD = 0, //ֹͣ���
    ONE_FIRE_CMD,      //����ģʽ
    RAPID_FIRE_CMD,    //����������ģʽ
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

/* �궨�� --------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/
void ConsoleTaskInit(void);
Console_t* Console_Pointer(void);

#endif
