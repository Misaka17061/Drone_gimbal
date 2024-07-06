/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "app_init.h"
#include "Buzzer/buzzer.h"
#include "imu_task.h"
#include "comm_task.h"
#include "timer_task.h"
#include "detect_task.h"
#include "gimbal_app.h"
#include "shoot_task.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/
void StartMusic(void);

/* ������ --------------------------------------------------------------------*/
void AppInit(void)
{
    BSP_Init();
	  //�ɸ�����Ҫ�������ã���������ǰ��CUBE-MX��������ã����BSP��.c�ļ�˵����

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
    Buzzer_SetBeep(HDO, 150);   //������	
    HAL_Delay(100);            //��ʱ
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
