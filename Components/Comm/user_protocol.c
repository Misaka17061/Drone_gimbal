/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "user_protocol.h"
#include "string.h"
#include "RemoteControl/remote_control.h"
#include "detect_task.h"
#include "gimbal_app.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/
Comm_VisionInfo_t vision_info;
Comm_RobotInfo_t robot_info;


/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/
/*************************************************
 * Function: UserProtocol_ParseHandler
 * Description: �û��Զ������ݽ�������
 * Input: cmd_id Э��������
 *        data ����ָ��
 *        len ���ݳ���
 * Return: ��
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


