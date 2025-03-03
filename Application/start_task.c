/* 包含头文件 ----------------------------------------------------------------*/
#include "start_task.h"
#include "cmsis_os.h"
#include "drone_gimbal_def.h"
#include "app_init.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/**
  * @brief  Function implementing the startTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartTask(void const*argument)
{
    AppInit();
    for(;;)
    {
        osDelay(START_TASK_PERIOD);
    }
}
