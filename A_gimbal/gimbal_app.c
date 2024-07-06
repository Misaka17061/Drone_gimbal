/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_app.h"
#include "drone_gimbal_def.h"
#include "app_init.h"

#include "comm_protocol.h"
#include "referee_system.h"
#include "user_protocol.h"

#include "timer_task.h"
#include "detect_task.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static TransmitHandle_t vision_tx_handle;
static uint8_t vision_tx_fifo_buffer[VISION_DATA_FIFO_SIZE];

static ReceiveHandle_t vision_rx_handle;
static uint8_t vision_rx_fifo_buffer[VISION_DATA_FIFO_SIZE];

/* 扩展变量 ------------------------------------------------------------------*/
GimbalHandle_t gimbal_handle;

/* 私有函数原形 --------------------------------------------------------------*/
static void Vision_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t Vision_RobotInfoUploadCallback(void *argc);
static void COM1_ReceiveCallback(uint8_t* data, uint16_t len);
static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len);
static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void COM2_ReceiveCallback(uint8_t* data, uint16_t len);

/* 函数体 --------------------------------------------------------------------*/
void GimbalAppConfig(void)
{
	
		gimbal_handle.yaw_motor.motor_info = GimbalMotorYaw_Pointer();
		gimbal_handle.pitch_motor.motor_info = GimbalMotorPitch_Pointer();
	
		//gimbal_handle.console = Console_Pointer();
    gimbal_handle.imu  = IMU_GetDataPointer();
    gimbal_handle.gimbal_can  = &can1_obj;
		gimbal_handle.ctrl_mode = GIMBAL_INIT;
	
		gimbal_handle.yaw_motor.offset_ecd = 5885;
    gimbal_handle.pitch_motor.offset_ecd =5300;
	
		gimbal_handle.yaw_motor.ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    gimbal_handle.pitch_motor.ecd_ratio = PITCH_MOTO_POSITIVE_DIR * PITCH_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    gimbal_handle.yaw_motor.max_relative_angle = 90;
    gimbal_handle.yaw_motor.min_relative_angle = -90;
    gimbal_handle.pitch_motor.max_relative_angle = 35;
    gimbal_handle.pitch_motor.min_relative_angle = -35;
	
    pid_init(&gimbal_handle.yaw_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             30.0f, 0.0f, 40.0f);/*调速*//*慢加P，抖加D*/ /*60,0,40*/
    pid_init(&gimbal_handle.yaw_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
             60.0f, 0.1f, 30.0f);/*调力*//**/
    pid_init(&gimbal_handle.pitch_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             45.0f, 0.0f, 10.0f);
    pid_init(&gimbal_handle.pitch_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 6000.0f,
             55.0f, 0.01f, 0.0f);
	
    OfflineHandle_Init(OFFLINE_VISION_INFO,             OFFLINE_WARNING_LEVEL,     1000,        0);
//    OfflineHandle_Init(OFFLINE_GIMBAL_PITCH,            OFFLINE_ERROR_LEVEL,       100,         0);
//    OfflineHandle_Init(OFFLINE_GIMBAL_YAW,              OFFLINE_ERROR_LEVEL,       100,         0);
//    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR1,   OFFLINE_ERROR_LEVEL,       100,         0);
//    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR2,   OFFLINE_ERROR_LEVEL,       100,         0);
//    OfflineHandle_Init(OFFLINE_TRIGGER_MOTOR,           OFFLINE_ERROR_LEVEL,       100,         0);
//	  OfflineHandle_Init(OFFLINE_DBUS,                    OFFLINE_WARNING_LEVEL,     100,         0);
//    OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,          OFFLINE_WARNING_LEVEL,     100,         0);
	
	
    Comm_TransmitInit(&vision_tx_handle, vision_tx_fifo_buffer, VISION_DATA_FIFO_SIZE, Vision_UploadDataHook);
	  SoftwareTimerRegister(Vision_RobotInfoUploadCallback, (void*)NULL, 10);
	
    Comm_ReceiveInit(&vision_rx_handle, VISION_PROTOCOL_HEADER_SOF, vision_rx_fifo_buffer, VISION_DATA_FIFO_SIZE, VisionProtocol_ParseHandler);
	
		BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
		BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
		BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
	  BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
		BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
}

static void Vision_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_UART_TransmitData(&com1_obj, data, len);
}

static int32_t Vision_RobotInfoUploadCallback(void *argc)
{
    Comm_RobotInfo_t* info = RobotInfo_Pointer();
    uint16_t robot_id = RefereeSystem_GetRobotID();
    info->data_head = 0xAA;
    if (robot_id > 100)     //ID大于100是蓝方  应该打红方；
    {
       info->enemy_color = Red;
    }
    else if (robot_id > 1)
    {
			 info->enemy_color = Blue;
    }
    else
    {
       info->enemy_color = AllColor;
    }
//      info->speed = sqrt(pow(console.chassis.vx,2) + pow(console.chassis.vy,2));
//      info->yaw_relative_angle = gimbal_handle.yaw_motor.sensor.relative_angle;
//      info->pitch_relative_angle = gimbal_handle.pitch_motor.sensor.relative_angle;
//      info->bullet_speed = shoot_handle.shooter_speed_limit;		
		  info->speed = 2;
			info->yaw_relative_angle = 10;
			info->pitch_relative_angle = 20;
		  info->bullet_speed = 30;
      info->data_tail = 0xA5;

     Comm_TransmitData(&vision_tx_handle, VISION_PROTOCOL_HEADER_SOF, VISION_DATA_CMD_ID, (uint8_t*)info, sizeof(Comm_RobotInfo_t));

                                                  //有改动，本为 ROBOT_DATA_CMD_ID ↑  
    return 0;
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)
{     
   Comm_ReceiveData(&vision_rx_handle, data, len);
}

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case GIMBAL_MOTOR_YAW_MESSAGE_ID:
        {
            Motor_DataParse(gimbal_handle.yaw_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_YAW);
        }break;
        case GIMBAL_MOTOR_PITCH_MESSAGE_ID:
        {
            Motor_DataParse(gimbal_handle.pitch_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_PITCH);
        }break;
        default:
            break;
    }
}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len)
{

}


static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case FRICTION_WHEEL_1_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_1_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR1);
        }break;
        case FRICTION_WHEEL_2_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_2_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR2);
        }break;
        case TRIGGER_MOTOR_MESSAGE_ID:
        {
            Motor_DataParse(TriggerMotor_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_TRIGGER_MOTOR);
        }break;
        default:
            break;
    }
}
