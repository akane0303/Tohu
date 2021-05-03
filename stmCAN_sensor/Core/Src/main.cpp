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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
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
CAN_HandleTypeDef     CanHandle;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
int                   TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
int wheeldirection[3]={0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Error_Handler(void);
//static HAL_StatusTypeDef CAN_Polling(void);
static void CAN_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t get_enc0(void){
	int16_t count=0;
	uint16_t encbuff=TIM1->CNT;
	TIM1->CNT=0;

	if(encbuff>32767){
		count=(int16_t)encbuff;
	}
	else{
		count=(int16_t)encbuff;
	}
	return count;
}
int16_t get_enc1(void){
	int16_t count=0;
	int16_t encbuff=TIM2->CNT;
	TIM2->CNT=0;
	if(encbuff>32767){
		count=(int16_t)encbuff;
	}
	else{
		count=(int16_t)encbuff;
	}
	return count;
}
int16_t get_enc2(void){
	int16_t count=0;
	int16_t encbuff=TIM3->CNT;
	TIM3->CNT=0;
	if(encbuff>32767){
		count=(int16_t)encbuff;
	}
	else{
		count=(int16_t)encbuff;
	}
	return count;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setbuf(stdout,NULL);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all p
   *
   *
   * eripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

  printf("%s","conf\n");
  CAN_Config();
  int cnt=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  wheeldirection[0]+=get_enc0();
	  wheeldirection[1]+=get_enc1();
	  wheeldirection[2]+=get_enc2();
	/*  const auto data0=static_cast<uint8_t>(wheeldirection[0]);
	  const auto data1=static_cast<uint8_t>(wheeldirection[1]);
	  const auto data2=static_cast<uint8_t>(wheeldirection[2]);
*/
	 if(wheeldirection[0]<0){
		 TxData[0]=(0x10000+wheeldirection[0])%0x100;
		 TxData[1]=(0x10000+wheeldirection[0])/0x100;
	 }
	 if(wheeldirection[0]>=0){
		 TxData[0]=wheeldirection[0]%0x100;
		 TxData[1]=wheeldirection[0]/0x100;
	 }
	 if(wheeldirection[1]<0){
		 TxData[2]=(0x10000+wheeldirection[1])%0x100;
		 TxData[3]=(0x10000+wheeldirection[1])/0x100;
	 }
	 if(wheeldirection[1]>=0){
		 TxData[2]=wheeldirection[1]%0x100;
		 TxData[3]=wheeldirection[1]/0x100;
	 }
	 if(wheeldirection[2]<0){
		 TxData[4]=(0x10000+wheeldirection[2])%0x100;
		 TxData[5]=(0x10000+wheeldirection[2])/0x100;
	 }
	 if(wheeldirection[2]>=0){
		 TxData[4]=wheeldirection[2]%0x100;
		 TxData[5]=wheeldirection[2]/0x100;
	 }

	/*  printf("%d",TxData[0]);
	  printf("%d",TxData[1]);
	  printf("%s","\n");
	  printf("%s","wheel");
*/
//	  printf("%d",wheeldirection[0]);
//	  printf("%s","\n");


	  printf("%d",wheeldirection[1]);
	  printf("%s","\n");

	//  printf("%d",wheeldirection[2]);
	//  printf("%s","\n");
	  if(HAL_CAN_AddTxMessage(&CanHandle,&TxHeader,TxData,&TxMailbox)!=HAL_OK){
		  printf("%s","txerror");
		  Error_Handler();
	  }
	 while(HAL_CAN_GetTxMailboxesFreeLevel(&CanHandle) != 3) {}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
HAL_StatusTypeDef CAN_Polling(void){
	CAN_FilterTypeDef  sFilterConfig;

	  /*##-1- Configure the CAN peripheral #######################################*/
	  /*
	  CanHandle.Instance = CAN2;

	  CanHandle.Init.TimeTriggeredMode = DISABLE;
	  CanHandle.Init.AutoBusOff = DISABLE;
	  CanHandle.Init.AutoWakeUp = DISABLE;
	  CanHandle.Init.AutoRetransmission = ENABLE;
	  CanHandle.Init.ReceiveFifoLocked = DISABLE;
	  CanHandle.Init.TransmitFifoPriority = DISABLE;
	  CanHandle.Init.Mode = CAN_MODE_NORMAL;
	  CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  CanHandle.Init.TimeSeg1 = CAN_BS1_15TQ;
	  CanHandle.Init.TimeSeg2 = CAN_BS2_4TQ;
	  CanHandle.Init.Prescaler = 3;

	  if(HAL_CAN_Init(&CanHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    //printf("%s","initerrror");
	  //}

	  /*##-2- Configure the CAN Filter ###########################################*/
/*
sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 14;

	  if(HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
	  {
	    /* Filter configuration Error */
	//    printf("%s","configerror");
	  //}

	  /*##-3- Start the CAN peripheral ###########################################*/
	  /*if (HAL_CAN_Start(&CanHandle) != HAL_OK)
	  {
	    /* Start Error */
	    //printf("%s","starterror");
	  //}

	  /*##-4- Start the Transmission process #####################################*/
	  /*
		TxHeader.StdId = 0x11;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.DLC = 2;
	  TxHeader.TransmitGlobalTime = DISABLE;
	  TxData[0] = 0xCA;
	  TxData[1] = 0xFE;

	  /* Request transmission */

	/*if(HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  {
	    /* Transmission request Error */
	  /*  printf("%s","transmiterror");
	  }

	  /* Wait transmission complete */
	  /*while(HAL_CAN_GetTxMailboxesFreeLevel(&CanHandle) != 3) {printf("%s","waiting");}

	  /*##-5- Start the Reception process ########################################*/
	 /*if(HAL_CAN_GetRxFifoFillLevel(&CanHandle, CAN_RX_FIFO0) != 1)
	  {
	    /* Reception Missing */
	   /* printf("%s","receptionerror");
	 /* }

	  if(HAL_CAN_GetRxMessage(&CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	  {
	    /* Reception Error */
	   /* printf("%s","rxerror");
	  }

	  if((RxHeader.StdId != 0x12)                     ||
	     (RxHeader.RTR != CAN_RTR_DATA)               ||
	     (RxHeader.IDE != CAN_ID_STD)                 ||
	     (RxHeader.DLC != 2)                          ||
	     ((RxData[0]<<8 | RxData[1]) != 0xCAFE))
	  {
	    /* Rx message Error */
	    /*return HAL_ERROR;
	  }

	  return HAL_OK; /* Test Passed */
//}
static void CAN_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = CAN2;

  CanHandle.Init.TimeTriggeredMode = DISABLE;
  CanHandle.Init.AutoBusOff = DISABLE;
  CanHandle.Init.AutoWakeUp = DISABLE;
  CanHandle.Init.AutoRetransmission = ENABLE;
  CanHandle.Init.ReceiveFifoLocked = DISABLE;
  CanHandle.Init.TransmitFifoPriority = DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  CanHandle.Init.TimeSeg1 = CAN_BS1_15TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_4TQ;
  CanHandle.Init.Prescaler = 3;

  if (HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 0;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&CanHandle) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  /*##-5- Configure Transmission process #####################################*/
  TxHeader.StdId = 0x322;
  TxHeader.ExtId = 0x02;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 6;
  TxHeader.TransmitGlobalTime = DISABLE;
}

/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
printf("%s","callback\n");
  if (HAL_CAN_GetRxMessage(&CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
	  printf("%s","rxerror");
    Error_Handler();
  }

  /* Display LEDx */
  if ((RxHeader.StdId == 0x321) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
  {
    printf("%s","success");
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
  printf("%s","errorhandler");
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
