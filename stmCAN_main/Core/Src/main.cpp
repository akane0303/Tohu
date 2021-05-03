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
#include"pure_pursuit.h"
#include"cubic_spline.h"
#include<vector>
#include<math.h>
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
uint8_t               TxData[8];
int                   RxData[8]={0,0,0,0,0,0,0,0};
uint32_t              TxMailbox;
double			        data[3]={0,0,0};

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
/*USER CODE END 0*/
void run1(int a){
	if(a>0){
		TIM1->CCR1=0;
		TIM1->CCR2=a;
	}
	else if(a<0){
		TIM1->CCR1=-1*a;
		TIM1->CCR2=0;
	}
	else{
		TIM1->CCR1=0;
		TIM1->CCR2=0;
	}
}
void run0(int a){
	if(a>0){
		TIM1->CCR3=-1*-1*a;
		TIM1->CCR4=0;
	}
	else if(a<0){
		TIM1->CCR3=0;
		TIM1->CCR4=-1*a;
	}
	else{
		TIM1->CCR3=0;
		TIM1->CCR4=0;
	}
}
void run2(int a){
	if(a>0){
		TIM2->CCR3=a;
		TIM2->CCR4=0;
	}
	else if(a<0){
		int b=-1*a;
		TIM2->CCR3=0;
		TIM2->CCR4=b;
	}
	else{
		TIM2->CCR3=0;
		TIM2->CCR4=0;
	}
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */
  double cnt[3]={0,0,0};
  double nowx=0,nowy=0,nowz=0;
  double speed_x,speed_y,v=0.1;
  double mt[3];
  double init_x=0,init_y=0,init_yaw=M_PI/2,init_v=0;
  double speed=0.1;
  uint8_t target_ind=0,last_ind=0;
  double lf=0;
  double delta=0;
  double circumference_length=0.1*M_PI;
  double Dist,Delta;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_USART2_UART_Init();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  CAN_Config();

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);

  std::vector<double>point_x{0,0,0,0,-0.5,-0.5,-1,-1.5,-2,-4,-5,-6,-7,-7.5,-8,-8,-8.5,-8.5,-8.5,-8.5};
  std::vector<double>point_y{0,1,2,3,3,2,1,0.6,0.5,0.5,0.5,0.5,0.6,1,2,3,3,2,1,0};
  std::vector<double>curvature;
  std::vector<double>rx;
  std::vector<double>ry;

  CubicSpline *course_x=new CubicSpline(point_x);
  CubicSpline *course_y=new CubicSpline(point_y);

  for(double i=0;i<point_x.size();i+=0.2){
	  rx.push_back(course_x->Calc(i));
	  ry.push_back(course_y->Calc(i));
  }
  for(int i=0.0;i<rx.size();i++){
       	  printf("%lf",rx[i]);
       	  printf("%s","\n");
  }


//曲率マップ生成---------------------------------
  Delta=atan2(ry[0],rx[0]);
  Dist=hypot(rx[0]-init_x,ry[0]-init_y);
  curvature.push_back(Delta/Dist);

  for(int i=0;i<rx.size()-1;i++){
	  Dist=hypot(rx[i+1]-rx[i],ry[i+1]-ry[i]);
	  Delta=atan2(ry[i+1],rx[i+1]);
	  curvature.push_back(Delta/Dist);
  }
 //------------------------------------------------

//------------------------------------------------------------------------------
  State state(init_x,init_y,init_yaw,init_v);
  last_ind=rx.size()-1;
  TargetCourse target_course(rx,ry);
  std::tie(target_ind,lf)=target_course.search_target_index(state);

  delete course_x;
  delete course_y;
  printf("%d",last_ind);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (last_ind>target_ind)
  {

	  if(HAL_GPIO_ReadPin(PC8_GPIO_Port,PC8_Pin)==0){

	  		  cnt[0]=data[0];
	  		  cnt[1]=data[1];
	  		  cnt[2]=data[2];

	  		  cnt[0]*=circumference_length;
	  		  cnt[1]*=circumference_length;
	  		  cnt[2]*=circumference_length;

	  		  nowx=-cnt[0]+(cnt[1]*sqrt(3)/2)+(cnt[2]*sqrt(3)/2);
	  		  nowy=-cnt[1]/2 +cnt[2]/2;
	  		  nowz=cnt[0]+cnt[1]+cnt[2];

	  		  std::tie(target_ind,delta)=pursuit_control(state,target_course,target_ind);
	  		  state.update(nowx,nowy,speed,delta);

	  		  speed_x=v*cos(delta);
	  		  speed_y=v*sin(delta);

	  		  mt[0]= -speed_x;
	  		  mt[1]= (speed_x/2)-(sqrt(3)/2)*speed_y;
	  		  mt[2]= (speed_x/2)+(sqrt(3)/2)*speed_y;

	  		  //d[0]=duty.calc(mt[0],p);
	  		  //d[1]=duty.calc(mt[1],p);
	  		  //d[2]=duty.calc(mt[2],p);

	  		  mt[0]*=1000*5;
	  		  mt[1]*=1000*5;
	  		  mt[2]*=1000*5;

	  		  //printf("%lf",curvature[target_ind]);
	  		  //printf("%s","\n");
	 		  if(curvature[target_ind]>20){
	  			  //printf("%s","curve\n");
	  			  mt[0]*=0.6;
	  			  mt[1]*=0.6;
	  			  mt[2]*=0.6;

	  		  }

	  		 run0(mt[0]);
			 run1(mt[1]);//モーター２
			 run2(mt[2]);//モーター３


			  printf("%s","d[0]");
			  printf("%lf",nowy);
	  		  printf("%s","\n");
	  		  printf("%s","d[1]");
	  		  printf("%lf",nowx);
	  		  printf("%s","\n");
	 		  printf("%s","d[2]");
	  		  printf("%lf",mt[2]);
	  		  printf("%s","\n");
	 		  printf("%s","delta");

	  		  printf("%lf",delta);
	  		  printf("%s","\n");

	  		  //HAL_Delay(10);


	  }
	  else{
		  run0(0);
		  run1(0);
		  run2(0);
	  }
	  	//  printf("%d",target_ind);
	  	//  printf("%s","\n");

	 /* if(HAL_CAN_AddTxMessage(&CanHandle,&TxHeader,TxData,&TxMailbox)!=HAL_OK){
		  printf("%s","txerror\n");
	  }
	//while(HAL_CAN_GetTxMailboxesFreeLevel(&CanHandle) != 3) {printf("%s","waiting");}
	//HAL_Delay(100);
   //printf("%s","b");



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  run0(0);//モーター１
  run1(0);//モーター２
  run2(0);//モーター３
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
    //Error_Handler();
	  printf("%s","initerro");
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
    //Error_Handler();
	  printf("%s","a");
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
   // Error_Handler();
  }

  /*##-5- Configure Transmission process #####################################*/
  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 2;
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
	//printf("%s","callback\n");
  if (HAL_CAN_GetRxMessage(&CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
	 printf("%s","rxmessageerror");
//    Error_Handler();
  }

  /* Display LEDx */
  if ((RxHeader.StdId == 0x322) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 6))
  {

    data[0]=RxData[2]+RxData[3]*0x100;
    	if(data[0]>0x8000){
    		data[0]-=0x10000;
    }
    data[1]=RxData[4]+RxData[5]*0x100;
        if(data[1]>0x8000){
        	data[1]-=0x10000;
     }
    data[2]=RxData[0]+RxData[1]*0x100;
     if(data[2]>0x8000){
        data[2]-=0x10000;
     }


   // printf("%s","success\n");
   //printf("%lf",data[0]);
   // printf("%s","\n");

   // printf("%d",data[1]);
   // printf("%s","\n");
   // printf("%d",data[2]);
   // printf("%s","\n");

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
