/* 包含头文件 ----------------------------------------------------------------*/
#include "app_init.h"
#include "Buzzer/buzzer.h"
#include "imu_task.h"
#include "comm_task.h"
#include "timer_task.h"
#include "detect_task.h"
#include "gimbal_app.h"
#include "shoot_task.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
void StartMusic(void);

/* 函数体 --------------------------------------------------------------------*/
void AppInit(void)
{
    BSP_Init();
	  //可根据需要加入配置（部分需提前在CUBE-MX内完成配置，详见BSP各.c文件说明）

    SoftwareTimerTaskInit();
    IMU_TaskInit();
    Comm_TaskInit();
    DetectTaskInit();
		GimbalAppConfig();
    ConsoleTaskInit();
		ShootTaskInit();
	
	//StartMusic();


}




void jbit(M x,uint16_t z)
{
	Buzzer_SetBeep(x,20);
	HAL_Delay(z);
	Buzzer_SetBeep(0,0);
	HAL_Delay(1);
}

void StartMusic(void)
{
    Buzzer_SetBeep(HDO, 150);   //蜂鸣器	
    HAL_Delay(100);            //延时
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
    Buzzer_SetBeep(HDO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
    Buzzer_SetBeep(bXI, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
	  Buzzer_SetBeep(HDO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(200);
	  Buzzer_SetBeep(SO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(200);
	Buzzer_SetBeep(SO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
		Buzzer_SetBeep(HDO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
		HAL_Delay(100);
		Buzzer_SetBeep(HFA, 150);
		HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
		HAL_Delay(100);
		Buzzer_SetBeep(HMI, 150);
		HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
		HAL_Delay(100);
		Buzzer_SetBeep(HDO, 150);
		HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
   
}
