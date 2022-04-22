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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "i2clcd.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
extern char ReceivedData[100];
extern uint8_t Rxcount;
extern uint32_t dataSize;
extern uint8_t check;
extern uint32_t time;

int OtoVao = 0, XeVao = 0, OtoRa = 0, XeRa = 0;
int TongOto = 20, TongXemay = 20,OtoCL,XMCL;
int lcdUpdate1=0,lcdUpdate2=0;
char dataRX[20],lcd[16];
uint8_t way1;
uint8_t way2;
uint8_t way3;
uint8_t way4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void USBTransmit(char *data);
void LCD1(void);
void LCD2(void);
void updateAll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void updateAll(void)
{
	lcdUpdate1=1; // Yeu cau truyen len LCD
	lcdUpdate2=1; // Yeu cau truyen len LCD
	sprintf(&dataRX[0],"OV%3d",OtoVao);
	USBTransmit(dataRX);
	HAL_Delay(50);
	sprintf(&dataRX[0],"OR%3d",OtoRa);
	USBTransmit(dataRX);
	HAL_Delay(50);
	sprintf(&dataRX[0],"XV%3d",XeVao);
	USBTransmit(dataRX);
	HAL_Delay(50);
	sprintf(&dataRX[0],"XR%3d",XeRa);
	USBTransmit(dataRX);
	HAL_Delay(50);
}
void USBTransmit(char *data)
	{
		CDC_Transmit_FS((uint8_t *)data, strlen(data));
	}
	void EXTI0_IRQHandler(void)
{
	if(way1) //Cham ca hai cam bien thi moi tang
		{
			way1=0;
			if(OtoCL>0)
			OtoVao++; //Tang o to vao
			OtoCL=TongOto-OtoVao+OtoRa;
			sprintf(&dataRX[0],"OV%3d",OtoVao);
			USBTransmit(dataRX);	
			lcdUpdate1=1;
		}
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
void EXTI1_IRQHandler(void)
{
	if(way2)
		{
			way2=0;
			if(OtoCL<TongOto)
			OtoRa++; //Tang o to ra
			OtoCL=TongOto-OtoVao+OtoRa;
			sprintf(&dataRX[0],"OR%3d",OtoRa);
			USBTransmit(dataRX);
			lcdUpdate1=1;
		}
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
void EXTI2_IRQHandler(void)
{
	if(way3)
		{
			way3=0;
			if(XMCL>0)
			XeVao++; //Tang xe may vao
			XMCL=TongXemay-XeVao+XeRa;
			updateAll();
			sprintf(&dataRX[0],"XV%3d",XeVao);
			USBTransmit(dataRX);
			lcdUpdate2=1;
		}
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
void EXTI3_IRQHandler(void)
{
	if(way4)
		{
			way4=0;
			if(XMCL<TongXemay)
			XeRa++; //Tang xe may ra
			XMCL=TongXemay-XeVao+XeRa;
			sprintf(&dataRX[0],"XR%3d",XeRa);
			USBTransmit(dataRX);
			lcdUpdate1=2;
		}		
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12))
	{
		way1=1;
		sprintf(&dataRX[0],"OV%3d",OtoVao);
		USBTransmit(dataRX);
	}
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))
	{
		way2=1;
		sprintf(&dataRX[0],"OR%3d",OtoRa);
		USBTransmit(dataRX);
	}
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14))
	{
		way3=1;
		sprintf(&dataRX[0],"XV%3d",XeVao);
		USBTransmit(dataRX);
	}
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))
	{
		way4=1;
		sprintf(&dataRX[0],"XR%3d",XeRa);
		USBTransmit(dataRX);
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}
void LCD1(void)
{
	//LCD ben oto
	lcdUpdate1=0;
	
	sprintf(&lcd[0],"TS:%3d  Vao:%3d",TongOto,OtoVao);
	lcd_goto_XY(1,0,&hi2c1); //Ghi dong 1
	lcd_send_string(lcd,&hi2c1); //Hien thi len LCD
	HAL_Delay(50);
	sprintf(&lcd[0],"Con:%3d  Ra:%3d",OtoCL,OtoRa);
	lcd_goto_XY(2,0,&hi2c1); //Ghi dong 2
	lcd_send_string(lcd,&hi2c1);
	HAL_Delay(50);
}
void LCD2(void)
{
	//LCD ben xe may
	lcdUpdate2=0;
	
	sprintf(&lcd[0],"TS:%3d  Vao:%3d",TongXemay,XeVao);
	lcd_goto_XY(1,0,&hi2c2);
	lcd_send_string(lcd,&hi2c2);
	HAL_Delay(50);
	sprintf(&lcd[0],"Con:%3d  Ra:%3d",XMCL,XeRa);
	lcd_goto_XY(2,0,&hi2c2);
	lcd_send_string(lcd,&hi2c2);
	HAL_Delay(50);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,500);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,500);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,500);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,500);
	//LCD khoi tao
	lcd_init(&hi2c1);
	lcd_init(&hi2c2);
	//USBTransmit("Init complete");
	LCD1();
	LCD2();
	OtoCL=TongOto-OtoVao+OtoRa;
	XMCL=TongXemay-XeVao+XeRa;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		if (lcdUpdate1) //Yeu cau update lcd
			LCD1();
		if (lcdUpdate2)
			LCD2();
		if  (check ==1 )  // Co du lieu nhan tu USB
		{
			if (strstr("Hello",ReceivedData)) //Kiem tra xem trong ReceivedData co "Hello" khong
				USBTransmit("Hello\n");
			else
			if (strstr("SV1ON",ReceivedData))
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1500); //Xoay servo 1 90 do
			else
			if (strstr("SV2ON",ReceivedData))
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,1500);
			else
			if (strstr("SV3ON",ReceivedData))
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,1500);
			else
			if (strstr("SV4ON",ReceivedData))
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,1500);
			else
			if (strstr("SV1OFF",ReceivedData))
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,500);
			else
			if (strstr("SV2OFF",ReceivedData))
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,500);
			else
			if (strstr("SV3OFF",ReceivedData))
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,500);
			else
			if (strstr("SV4OFF",ReceivedData))
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,500);
			if (strstr("UP",ReceivedData))
				updateAll();
			for(int i = 0; i < dataSize; i++)
			{
				ReceivedData[i] = 0; // Xoa du lieu nhan
			}
			check = 0;
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
