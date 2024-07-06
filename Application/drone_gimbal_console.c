/* 包含头文件 ----------------------------------------------------------------*/
#include "drone_gimbal_console.h"
#include "shoot_task.h"
#include "drone_gimbal_def.h"
#include "cmsis_os.h"
#include "RemoteControl/remote_control.h"
#include "user_protocol.h"
#include "detect_task.h"
#include "ramp.h"
#include "math.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId ConsoleTaskHandle;
RC_Switch_t wheel_switch;
RC_Info_t last_rc;

/* 扩展变量 ------------------------------------------------------------------*/
Console_t console;
extern GimbalHandle_t gimbal_handle;

/* 私有函数原形 --------------------------------------------------------------*/
static void RemoteControlWheelAction(void);
static void RemoteControl_Operation(void);
static void Keyboard_Operation(void);
static void Other_Operation(void); 

/* 函数体 --------------------------------------------------------------------*/
void ConsoleTask(void const*argument)
{
    for(;;)
    {
        RemoteControlWheelAction();
        switch (console.ctrl_mode)
        {
            case PREPARE_MODE:
            {
                if(gimbal_handle.ctrl_mode != GIMBAL_INIT
                        && gimbal_handle.ctrl_mode != GIMBAL_RELAX)
                {
                    console.ctrl_mode = NORMAL_MODE;
                    console.gimbal_cmd = GIMBAL_NORMAL_CMD;
                    console.shoot_cmd = SHOOT_STOP_CMD;
                }
                else
                {
                    console.gimbal_cmd  = GIMBAL_INIT_CMD;
                    console.shoot_cmd = SHOOT_STOP_CMD;
                }
            }break;
            case NORMAL_MODE:
            {
                if (console.rc->sw1 == REMOTE_SWITCH_VALUE_CENTRAL)
                {
                    RemoteControl_Operation();
                }
                else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_UP)
                {
                    Keyboard_Operation();
                }
                else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_DOWN)
                {
                    Other_Operation();
                }
            }break;
            case SAFETY_MODE:
            {
                if(!CheckDeviceIsOffline(OFFLINE_DBUS))
                {
                    console.ctrl_mode = PREPARE_MODE;
                }
                else
                {
                    console.gimbal_cmd  = GIMBAL_RELEASE_CMD;
                    console.shoot_cmd = SHOOT_RELEASE_CMD;
                }
            }break;
            default:
                break;
        }
        
        if(CheckDeviceIsOffline(OFFLINE_DBUS))
        {
            console.ctrl_mode = SAFETY_MODE;
        }
        last_rc = *console.rc;
        osDelay(CONSOLE_TASK_PERIOD);
    }
}

void ConsoleTaskInit(void)
{
	
    console.rc = RC_GetDataPointer();
    console.ctrl_mode = PREPARE_MODE;
    console.gimbal_cmd = GIMBAL_INIT_CMD;
    console.shoot_cmd = SHOOT_STOP_CMD;
    console.shoot.fire_cmd = STOP_FIRE_CMD;

    
    osThreadDef(console_task, ConsoleTask, osPriorityNormal, 0, 256);
    ConsoleTaskHandle = osThreadCreate(osThread(console_task), NULL);
}

static void RemoteControl_Operation(void)
{
    static uint32_t shoot_time = 0; 

    if (console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)
    {
        console.gimbal_cmd = GIMBAL_VISION_AIM_CMD;   

        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
		
		else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)
		{
				console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
		}
		
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;

        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_FALSE)
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;

        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
		
    if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }	
    }
    else if (console.shoot_cmd == SHOOT_START_CMD)
    {         
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_STOP_CMD;
        }
        else if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO2)
        {
            console.shoot.fire_cmd = ONE_FIRE_CMD;
        }
        else if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
        {
            shoot_time++;
            if(shoot_time > 50)
                console.shoot.fire_cmd = ONE_FIRE_CMD ;
            else
               console.shoot.fire_cmd = STOP_FIRE_CMD;
        }
          else
           {
            console.shoot.fire_cmd = STOP_FIRE_CMD;
            shoot_time = 0;
           }
    
   }
}

static void Keyboard_Operation(void)
{

}

static void Other_Operation(void)
{

}

static void RemoteControlWheelAction(void)
{
    static uint8_t wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    if (console.rc->wheel < -440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_UP;
    }
    else if (console.rc->wheel > -220 && console.rc->wheel < 220)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    }
    else if (console.rc->wheel > 440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_DOWN;
    }
    RC_SwitchAction(&wheel_switch, wheel_sw);
		
}

Console_t* Console_Pointer(void)
{
    return &console;
}
