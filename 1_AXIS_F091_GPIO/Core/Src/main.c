/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* CODE
TMC5160
- SPI interface
- GPIO DRV_ENN (Low = Motor On, High = Motor Off)
- GPIO STEP
- GPIO DIR

40 bit SPI interface (5 x 8 bit message)
1st 8 bits determine Read / Write and Adress
2nd to 5th 8 bits are Data


AS5055A
- SPI interface
- INT GPIO (optional)

TJA1042TK_3_1J
- CAN interface (CAN1 SEND 0x01 , if CAN2 receives 0x01 , send back 0x02. if CAN1 receives 0x02 , send back 0x01, repeat)
- GPIO CAN_STB (Low = Normal , High = Standby)  CHECK

EXT_OUT (24V)
- GPIO EXT_OUT_1
- GPIO EXT_OUT_2

EXT_IN (24V tolerant)
- GPIO REFL_UC
- GPIO REFR_UC

ADC_IN (5V tolerant)
- AIN AIN_MCU


To use the TMC-API, perform the following steps in your code:
- Implement the tmcXXXX_readWriteArray() function in in your code. This function provides the necessary hardware access to the TMC-API.

- Call tmcXXXX_init() once for each Trinamic IC in your design. This function initializes an IC object which represents one physical IC.

- Call tmcXXXX_periodicJob() periodically. Pass a millisecond timestamp as the tick parameter.

- After initializing, calling tmcXXXX_reset() or tmcXXXX_restore(), the TMC-API will write multiple registers to the IC (referred to as IC configuration).
Per call to tmcXXXX_periodicJob(), one register will be written until IC configuration is completed.
Once the IC configuration is completed, you can use tmcXXXX_readInt() and tmcXXXX_writeInt() to read and write registers.
*/


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc/ic/TMC5160/TMC5160.h"
#include "AS5055A_Registers.h"
//#include "SSM1ST2420HC.c"

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
 ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
TMC5160TypeDef h5160;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void TMC5160_Basic_Init(void);
void TMC5160_Basic_Rotate(uint8_t Mode);
void TMC5160_Rotate_To(uint32_t Position);
void TMC5160_Stop(void);
uint32_t TMC5160_SPIWrite(uint8_t Adress, uint32_t Value, int Action);

void AMS5055_Basic_Init(void);
uint16_t AMS5055_SPIWriteInt(uint16_t Adress, int Action);
uint16_t AMS5055_Get_Position(void);
uint8_t AMSParity(uint16_t value);
void AMS_Delay(uint32_t Delay);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


CAN_TxHeaderTypeDef CANTxHeader;
CAN_RxHeaderTypeDef CANRxHeader;
ConfigurationTypeDef configHeader;

uint8_t CANTxData[8];	//TX data array CAN
uint8_t CANRxData[8];	//RX data array CAN
uint32_t TxMailbox[3];		//CAN Mailbox

// SPI1 = AS5055A 16 bit data
uint8_t SPI1TxData[2];	//TX data array SPI1
uint8_t SPI1RxData[2];	//RX data array SPI1
uint16_t SPI1Rx = 0;
uint16_t SPI1Tx = 0;
uint16_t Angle;
uint16_t Error;
uint16_t Angles[100];

// SPI2 = TMC5160 40 bit data
uint8_t SPI2TxData[5];  //TX data array SPI2
uint8_t SPI2RxData[5];  //RX data array SPI2
uint32_t SPI2Rx = 0;

uint16_t ADCReadings[1000];
int s = 0;


int Datacheck;
int ReadingData = 0;
int count = 0;
int Ax = 0;
int x;

/* github test 19-6-23 */
/* i think its working. */


/*  CAN RECEIVE INTERRUPT */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CANRxHeader, CANRxData);

	if (CANRxData[0] == 0x01)
	{
		Datacheck = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == AMS_INT_Pin && ReadingData == 0)
	{
		ReadingData = 1;


		if(Ax == 0) // Do not write, first value is random
		{
			AMS5055_Get_Position();
		}

		else
		{
			Angles[Ax] = AMS5055_Get_Position();
		}

		if(Ax == 100)
		{
			Ax = 0;
		}

		else
		{
			Ax++;
		}

		ReadingData = 0;
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
  MX_ADC_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  TMC5160_Stop();
  HAL_GPIO_WritePin(GPIOA,DRV_ENN_Pin,1); // LOW = ON
  HAL_Delay(2500);	//startup delay, so motor does not spin on debug

  HAL_GPIO_WritePin(GPIOB,TMC_CS_Pin,1); // set TMC CS high

  /* Perform Basic Init of TMC5160 and AMS5055 */
  TMC5160_Basic_Init();
  AMS5055_Basic_Init();

  /* Enable DRV stage*/
  HAL_GPIO_WritePin(GPIOA,DRV_ENN_Pin,0); // LOW = ON
  HAL_Delay(10);

  //Read IN1 (5 to 24V)
  while(HAL_GPIO_ReadPin(GPIOB, REFL_UC_Pin) == 1)
  {
  }

  //Read IN2 (5 to 24V)
  while(HAL_GPIO_ReadPin(GPIOB, REFR_UC_Pin) == 1)
  {
  }

  /*

  //Read IN1 (5 to 24V)
  while(HAL_GPIO_ReadPin(GPIOB, REFL_UC_Pin) == 1)
  {
  }

  //Read IN2 (5 to 24V)
  while(HAL_GPIO_ReadPin(GPIOB, REFR_UC_Pin) == 1)
  {
  } */

  // Enable OUT2 for 1 sec (24V)
  HAL_GPIO_WritePin(GPIOB,EXT_OUT_2_Pin,1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB,EXT_OUT_2_Pin,0);

  //Enable OUT1 for 1 sec (24V)
  HAL_GPIO_WritePin(GPIOB,EXT_OUT_1_Pin,1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB,EXT_OUT_1_Pin,0);

/* Analog read check, AIN max = 5V , circuit has voltage devider for 3V max.

  while(s <= 100)
  {
  	  HAL_ADC_Start(&hadc);
  	  HAL_ADC_PollForConversion(&hadc, 10);
  	  ADCReadings[s] = HAL_ADC_GetValue(&hadc);
  	  s++;
   	  HAL_Delay(10);
  }

  HAL_ADC_Stop(&hadc);
  s = 0;
 */


  HAL_GPIO_WritePin(GPIOA,DRV_ENN_Pin,1); // LOW = ON
  TMC5160_Stop();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	//HAL_GPIO_WritePin(GPIOA,CAN_STB_Pin,0); // Set STB pin LOW for normal operation
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */


  /* CAN filter */

  CAN_FilterTypeDef  sFilterConfig;

  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 10;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CAN_STB_Pin|AMS_CS_Pin|DRV_ENN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin|EXT_OUT_1_Pin|EXT_OUT_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN_STB_Pin AMS_CS_Pin DRV_ENN_Pin */
  GPIO_InitStruct.Pin = CAN_STB_Pin|AMS_CS_Pin|DRV_ENN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AMS_INT_Pin */
  GPIO_InitStruct.Pin = AMS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AMS_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TMC_CS_Pin EXT_OUT_1_Pin EXT_OUT_2_Pin */
  GPIO_InitStruct.Pin = TMC_CS_Pin|EXT_OUT_1_Pin|EXT_OUT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : REFL_UC_Pin REFR_UC_Pin */
  GPIO_InitStruct.Pin = REFL_UC_Pin|REFR_UC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
/*
void tmc5160_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
	HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0);
	//HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,uint32_t Timeout);
	HAL_SPI_TransmitReceive(&hspi2, &data, &data, length, 100);
	HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1);
}*/


uint32_t TMC5160_SPIWrite(uint8_t Adress, uint32_t Value, int Action)
{

	  SPI2Rx = 0;
	  HAL_GPIO_WritePin(GPIOB,TMC_CS_Pin,0); // set TMC CS low

	  if (Action == 1) //Write
	  {
		SPI2TxData[0] = Adress + 0x80;
	  }

	  else //Read
	  {
		SPI2TxData[0] = Adress;
	  }

	  SPI2TxData[1] = Value >> 24;
	  SPI2TxData[2] = Value >> 16;
	  SPI2TxData[3] = Value >> 8;
	  SPI2TxData[4] = Value;

	  HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100);


	  SPI2Rx += (SPI2RxData[1] << 24);
	  SPI2Rx += (SPI2RxData[2] << 16);
	  SPI2Rx += (SPI2RxData[3] << 8);
	  SPI2Rx += (SPI2RxData[4] << 0);

	  HAL_GPIO_WritePin(GPIOB,TMC_CS_Pin,1); // set TMC CS high

	  return SPI2Rx;
}


uint16_t AMS5055_SPIWriteInt(uint16_t Adress, int Action)
{
	/* WRITE = 0
	| 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01  00 |
 	| 0	|			 Data 14:1		   		      | PAR|


 	(Data (14 bit) +  + parity)
	| 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01  00 |
 	|		 		 Data 15:2		   		   |EF| PAR|

 	EF-> 0 = no command frame error occurred, 1 = error occurred
 	PAR -> Parity bit
	*/

	/* READ = 1
	| 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01  00 |
 	| 1	|			 Data 14:1		   		      | PAR|



	| 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01  00 |
 	|				 Data 15:2		   		   |DC| PAR|

 	DC -> Don't care
 	PAR -> Parity bit
	*/

	  SPI1Rx = 0;
	  SPI1Tx = 0;
	  SPI1Tx = (Adress << 1);

	  if (Action == 1) //READ
	  {
		  SPI1Tx = SPI1Tx | 0x8000;
	  }

	  SPI1Tx = SPI1Tx | AMSParity(SPI1Tx);

	  HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low

	  SPI1TxData[0] = SPI1Tx >> 8;
	  SPI1TxData[1] = SPI1Tx;

	  HAL_SPI_TransmitReceive(&hspi1, SPI1TxData, SPI1RxData, 0x02, 100);

	  HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high

	  SPI1Rx += (SPI1RxData[0] << 8);
	  SPI1Rx += (SPI1RxData[1] << 0);

	  return SPI1Rx;
}

void AMS_Delay(uint32_t Delay)
{
	  uint32_t tickstart = HAL_GetTick();
	  uint32_t wait = Delay;

	  /* Add a freq to guarantee minimum wait */
	  if (wait < HAL_MAX_DELAY)
	  {
	    wait += (uint32_t)(uwTickFreq);
	  }

	  while((HAL_GetTick() - tickstart) < wait)
	  {
		  AMS5055_SPIWriteInt(ANGULAR_DATA,1);
		  HAL_Delay(10);
		  x++;

		  if (x == 1000)
		  {
			  x = 0;
		  }
	  }
}


void AMS5055_Basic_Init(void)
{
	Angles[Ax] = AMS5055_SPIWriteInt(ANGULAR_DATA,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(NOP,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(AGC,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(SOFTWARE_RESET,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(SOFTWARE_RESET_SPI,0); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(ANGULAR_DATA,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(NOP,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(AGC,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(ANGULAR_DATA,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(ANGULAR_DATA,1); // Random
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(ANGULAR_DATA,1); // Random
	Ax++;
	HAL_Delay(100);







	/*
	Angles[Ax] = AMS5055_SPIWriteInt(NOP,1); // Ang data
	Ax++;
	HAL_Delay(100);

	Angles[Ax] = AMS5055_SPIWriteInt(NOP,1); // 0
	Ax++;
	HAL_Delay(100);
	*/
}

uint8_t AMSParity(uint16_t value)
{
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}

		value >>= 1;
	}
	return cnt & 0x1;
}


uint16_t AMS5055_Get_Position(void)
{
	Angle = AMS5055_SPIWriteInt(NOP,1);
	//Angle = AMS5055_SPIWriteInt(ANGULAR_DATA,1);

	/*
	if (((Angle >> 1) & 0x1))
	{
		AMS5055_SPIWriteInt(CLEAR_ERROR_FLAG); //clear error flag
	}
	*/

	//Angle = Angle << 2;
	//Angle = (Angle >> 2) & 0x3fff;
	//Angle = ((Angle * 360) / 4095);

	return Angle;
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
