/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
 #include "queue.h"
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

/* USER CODE BEGIN PV */
 QueueHandle_t Test_Queue = NULL;

#define QUEUE_LEN 4 /* 队列的长度，最大可包含多少个消息 */
#define QUEUE_SIZE 4 /* 队列中每个消息大小（字节） */

//任务句柄
static TaskHandle_t AppTaskCreate_Handle;

static TaskHandle_t Send_Task_Handle = NULL;
static TaskHandle_t Receive_Task_Handle = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void AppTaskCreate(void);/* 用于创建任务 */

static void Send_Task(void* pvParameters);/* Send_Task 任务实现 */
static void Receive_Task(void* pvParameters);/* Receive_Task 任务实现 */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	BaseType_t xReturn = pdPASS;/* 定义�?个创建信息返回�?�，默认�? pdPASS */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
//  while(1)
//  {
//	  if (HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_SET)
//	  {
//		  HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_SET);
//	  }
//	  else
//	  {
//		  HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_RESET);
//	  }
//  }

  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,
                        (const char* )"AppTaskCreate",//任务名称
                        (uint32_t )128, //任务堆栈大小
                        (void* )NULL,//传�?�给任务函数的参�?
                        (UBaseType_t )1, //任务优先�?
						(TaskHandle_t* )&AppTaskCreate_Handle);
  
  if (pdPASS == xReturn)
	  vTaskStartScheduler(); /* 启动任务，开启调�? */
  else
	  return -1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void AppTaskCreate(void)
{
	BaseType_t xReturn = pdPASS;/* 定义�?个创建信息返回�?�，默认�? pdPASS */

    taskENTER_CRITICAL(); //进入临界�?

    Test_Queue = xQueueCreate((UBaseType_t ) QUEUE_LEN,/* 消息队列的长度 */
    		(UBaseType_t ) QUEUE_SIZE);/* 消息的大小 */

    if (NULL == Test_Queue)
    	while(1);

    xReturn = xTaskCreate((TaskFunction_t )Receive_Task,/* 任务入口函数 */
							(const char* )"Receive_Task",/* 任务名字 */
    		(uint16_t )512, /* 任务栈大小 */
			(void* )NULL, /* 任务入口函数参数 */
			(UBaseType_t )2, /* 任务的优先级 */
			(TaskHandle_t* )&Receive_Task_Handle);/*任务控制块指针*/

    if (pdPASS != xReturn)
    	while(1);

    xReturn = xTaskCreate((TaskFunction_t )Send_Task,/* 任务入口函数 */
							(const char* )"Receive_Task",/* 任务名字 */
    		(uint16_t )512, /* 任务栈大小 */
			(void* )NULL, /* 任务入口函数参数 */
			(UBaseType_t )3, /* 任务的优先级 */
			(TaskHandle_t* )&Send_Task_Handle);/*任务控制块指针*/

    if (pdPASS != xReturn)
        	while(1);

    vTaskDelete(AppTaskCreate_Handle); //删除 AppTaskCreate 任务
    
    taskEXIT_CRITICAL(); //�?出临界区
}

static void Send_Task(void* parameter)
{
	BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为 pdPASS */
	uint32_t send_data1 = 1;
	uint32_t send_data2 = 2;

	while (1)
	{
		if (HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_SET)
		{
			xReturn = xQueueSend( Test_Queue,&send_data1, 0);
		}
		else
		{
			xReturn = xQueueSend( Test_Queue,&send_data2, 0);
		}

		vTaskDelay(20);/* 延时 20 个 tick */

	}
}

static void Receive_Task(void* parameter)
{
	BaseType_t xReturn = pdTRUE;/* 定义一个创建信息返回值，默认为 pdTRUE */
	uint32_t r_queue; /* 定义一个接收消息的变量 */

	while (1)
	{
		xReturn = xQueueReceive( Test_Queue, /* 消息队列的句柄 */
								&r_queue, /* 接受的消息内容 */
								portMAX_DELAY); /* 等待时间 一直等 */

		if (xReturn == pdTRUE)
		{
			if (r_queue == 2)
			{
				HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_RESET);
			}
		}
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
