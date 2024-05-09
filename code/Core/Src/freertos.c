/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "MPU6050.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static MPU6050_Data_Struct MPU_Data;
static int16_t encoder_left,encoder_right;

/* USER CODE END Variables */
/* Definitions for Task_MPU_5ms */
osThreadId_t Task_MPU_5msHandle;
const osThreadAttr_t Task_MPU_5ms_attributes = {
  .name = "Task_MPU_5ms",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_PID_20ms */
osThreadId_t Task_PID_20msHandle;
const osThreadAttr_t Task_PID_20ms_attributes = {
  .name = "Task_PID_20ms",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_OLED_50ms */
osThreadId_t Task_OLED_50msHandle;
const osThreadAttr_t Task_OLED_50ms_attributes = {
  .name = "Task_OLED_50ms",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTask_MPU_5ms(void *argument);
void StartTask_PID_20ms(void *argument);
void StartTask_OLED_50ms(void *argument);

/* USER CODE END FunctionPrototypes */


void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_MPU_5ms */
  Task_MPU_5msHandle = osThreadNew(StartTask_MPU_5ms, NULL, &Task_MPU_5ms_attributes);

  /* creation of Task_PID_20ms */
  //Task_PID_20msHandle = osThreadNew(StartTask_PID_20ms, NULL, &Task_PID_20ms_attributes);

  /* creation of Task_OLED_50ms */
  Task_OLED_50msHandle = osThreadNew(StartTask_OLED_50ms, NULL, &Task_OLED_50ms_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask_MPU_5ms */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_MPU_5ms */
void StartTask_MPU_5ms(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		while (MPU_GetData(&MPU_Data)) {}
		MPU_Get_Accelerometer(&MPU_Data);
		MPU_Get_Gyroscope(&MPU_Data);
		//encoder_left=Motor_getSpeed(MOTOR_A);
		//encoder_right=Motor_getSpeed(MOTOR_B);//读取encode_left, encode_right，读取的周期就是本函数的运行周期，不需要额外设置
		osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StarttTask_PID_20ms */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StarttTask_PID_20ms */
void StartTask_PID_20ms(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	
	static uint32_t Balance_Pwm;
	static float Velocity_Pwm,Turn_Pwm;
	
    Balance_Pwm =  Motor_verticalRingPD(MPU_Data.pitch,  MPU_Data.gyro_y); //平衡环PD计算
    Velocity_Pwm = Motor_verticalSpeedPI( encoder_left,  encoder_right,MPU_Data.pitch, 10.0);//速度环PI计算
    Turn_Pwm = Motor_verticalTurnPD(0,MPU_Data.yaw,  MPU_Data.gyro_z);//转向环PD计算
    int Moto1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;//计算左轮电机所需PWM
    int Moto2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;//计算右轮电机所需PWM
    Motor_PWMLimiting(&Moto1,&Moto2);//对最终给两个电机的PWM进行限幅
	if(MPU_Data.pitch < -40 || MPU_Data.pitch > 40) 			{Moto1=0;Moto2=0;}
    Motor_setSpeed(MOTOR_A,Moto1);
	Motor_setSpeed(MOTOR_B,Moto2);
		osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}
/* USER CODE BEGIN Header_StartTask_OLED_50ms */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_OLED_50ms */
void StartTask_OLED_50ms(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	    OLED_showSignedNum(2, 8, MPU_Data.pitch, 3);
		OLED_showSignedNum(3, 8, MPU_Data.roll , 3);
		OLED_showSignedNum(4, 8, MPU_Data.yaw, 3);
		osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

