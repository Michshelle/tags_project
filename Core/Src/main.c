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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MFRC522.h"
#define	uchar	unsigned char
#define	uint	unsigned int
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
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char get;
uchar getdata[16];
int l,ii;
uchar senddata[]={52,49,46,46,70,24,0,0,57,41,56,45,66,65,74,66};
char reset[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

byte keyA[6] = {0xFf, 0xFf, 0xFF, 0xFF, 0xFF, 0xFF, };
byte keyB[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void SPI_mfrc_Init(void);
void delay(int d);

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
	uchar process, status, checksum1, str[MAX_LEN],cap,findex=0,mystr[5];

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
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  SPI_mfrc_Init();
  MFRC522_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GPIOC->ODR |= 1<<4;
  HAL_UART_Transmit(&huart2,(uchar *)"\n\rSerial Connected...\n\r", 25, 5000);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     findex++;

     // Find cards
		
     status = MFRC522_Request(PICC_REQIDL, str); 
  
     if (status == MI_OK) {			   
       HAL_UART_Transmit(&huart2,(uchar *)"\n\r~~~ Card detected!\n\r", 24, 5000);
       HAL_UART_Transmit(&huart2,(uchar *)"Card Type : \n\r", 16, 5000);
       HAL_UART_Transmit(&huart2,(uchar *)&str[0],8, 5000); 
     }
		 //else
			 //HAL_UART_Transmit(&huart2,(uchar *)"Err1\n\r", 6, 5000);
       
     // Anti-collision, return card serial number == 4 bytes
     status = MFRC522_Anticoll(str);
     if (status == MI_OK) {
			   
     checksum1 = str[0] ^ str[1] ^ str[2] ^ str[3];
     HAL_UART_Transmit(&huart2,(uchar *)"\n\rThe card's number is:\n\r", 25, 5000);
	   sprintf((char *)mystr,"%3d:%3d:%3d:%3d\n\r",(uchar)str[0],(uchar)str[1],(uchar)str[2],(uchar)str[3]);
	   HAL_UART_Transmit(&huart2,mystr, 18, 5000);
     cap=MFRC522_SelectTag(str);
     //SER_putString (0,"\n\rcap:");
     //SER_SendChar(0,cap);


     /// Be Carefull !! :: 
     ///write data on 0,1,2,4,5,6,8,... and Don't write data on 3,7,11,.... blocks .
     /// write Key on 3,7,11,... blocks. 


     // Read from and Wirte to block 1 and 2 ////

      for (ii = 1; ii < 3; ii++) {  

       // Try to authenticate each block first with the A key.
	       process=MFRC522_Auth(0x60, ii,keyA ,str);
      	 if (process == MI_OK) {
         		process= MFRC522_Write(ii,senddata);
            if (process == MI_OK) {
	            process= MFRC522_Read(ii, getdata);
	           	if (process == MI_OK) 
	                  HAL_UART_Transmit(&huart2,(uchar *)"\n\r block\n\r", 10, 5000);
              //		SER_SendChar(0,ii);
              //		SER_putString (0,":");
            		  for (l=0;l<16;l++){
                    HAL_UART_Transmit(&huart2,(uchar *)&getdata[l], 1, 5000);																		
                    HAL_UART_Transmit(&huart2,(uchar *)":", 1, 5000);	  
	  	   }						  
	   
      } 
	    }else{
    	  process=MFRC522_Auth(0x61, ii,keyB,str);
	      if (process == MI_OK) {
   				process= MFRC522_Write(ii,senddata);
      	  if (process == MI_OK) {
	            process= MFRC522_Read(ii, getdata);
	        	  if (process == MI_OK) 
	         		  for (l=0;l<16;l++){
	   	             HAL_UART_Transmit(&huart2,(uchar *)&getdata[l], 1, 5000);																		
                   HAL_UART_Transmit(&huart2,(uchar *)":", 1, 5000);	  
					   }						  
	   		  } 
          }else
  		    {
              HAL_UART_Transmit(&huart2,(uchar *)"\n\rKeys are wrong.\n\r", 19, 5000);
		      }
	      }
	    }
      HAL_UART_Transmit(&huart2,(uchar *)"\n\rEnd.\n\r", 8, 5000);
   	  MFRC522_Halt();
	    MFRC522_Init();
			
	    HAL_Delay(2000);
	  }	
		// else
			// HAL_UART_Transmit(&huart1,(uchar *)"Err2\n\r", 6, 5000);
     
   // sprintf((char *)mystr,"%3d\r",findex);
	//	HAL_UART_Transmit(&huart1,mystr, 4, 5000);
					 
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	

  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void delay(int d){
  int i;
for (i=0;i<d*10000;i++);
}


void SPI_mfrc_Init(void)
{
	CS_PORT_NAME->ODR |= 1<<CS_PIN_NUM;  
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
