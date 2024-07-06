/* 包含头文件 ----------------------------------------------------------------*/
#include "user_protocol.h"
#include "string.h"
#include "RemoteControl/remote_control.h"
#include "detect_task.h"
#include "gimbal_app.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
Comm_VisionInfo_t vision_info;
Comm_RobotInfo_t robot_info;


/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: UserProtocol_ParseHandler
 * Description: 用户自定义数据解析处理
 * Input: cmd_id 协议命令码
 *        data 数据指针
 *        len 数据长度
 * Return: 无
*************************************************/

void VisionProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len)
{
    switch(cmd_id)
    {
        case VISION_DATA_CMD_ID:
        {  
            memcpy(&vision_info, data, sizeof(Comm_VisionInfo_t));
            OfflineHandle_TimeUpdate(OFFLINE_VISION_INFO);
            
        }break;
				default:
        break;
    }
}

Comm_VisionInfo_t* VisionInfo_Pointer(void)
{

    return &vision_info;
}

Comm_RobotInfo_t* RobotInfo_Pointer(void)
{
    return &robot_info;
}


