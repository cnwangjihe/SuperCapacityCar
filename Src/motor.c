#include "motor.h"
#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void StartMotorTask(void const * argument)
{
  for(;;)
  {
    osDelay(10);
  }
}