#ifndef	USER_PROTOCOL_H
#define USER_PROTOCOL_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "comm_protocol.h"
#include "gimbal_app.h"
#include	<stdbool.h>

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum
{
    RC_DATA_CMD_ID           = 0x0001,
} USER_CMD_ID_e;

typedef enum
{
    VISION_DATA_CMD_ID          = 0x0001,
    ROBOT_DATA_CMD_ID           = 0x0002,
	ATTACK_DATA_CMD_ID          = 0x0000,
} VISION_CMD_ID_e;

typedef enum
{
    Gray = 0,
    Red = 1,
    Blue = 2,
    AllColor = Red|Blue
} EnemyColor_e;

typedef struct
{
    uint8_t data_head;    //ͷ֡ Ĭ��0xAA
		float pitch;
		float yaw;
		float yaw_speed;
		bool canshoot;
		bool switch_;
		bool enemy_move_state;
		uint8_t dataend;
		uint16_t crc16;
} Comm_VisionInfo_t;

typedef struct
{
   	uint8_t data_head;     //VisionState_e
	  uint8_t enemy_color;
	  fp32    speed;
	  fp32    yaw_relative_angle;
	  fp32    pitch_relative_angle;
	  uint16_t bullet_speed;
    uint8_t  data_tail; 
} Comm_RobotInfo_t;

/* �궨�� --------------------------------------------------------------------*/
#define USER_PROTOCOL_HEADER_SOF     0xAA
#define VISION_PROTOCOL_HEADER_SOF   0x55
#define VISION_DATA_FIFO_SIZE        (256u)

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/
void VisionProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
void UserProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
Comm_VisionInfo_t* VisionInfo_Pointer(void);
Comm_RobotInfo_t* RobotInfo_Pointer(void);

#endif
