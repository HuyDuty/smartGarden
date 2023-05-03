/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* VARIABLES SAVE VALUE OF SENSOR*/
int Temperature = 0, Humidity = 0;
int Soil = 0, Light = 0;
char statusRain = '0';
/* VARIABLES SAVE STATUS OF DEVICE */
char mode = '1';
int statusPump = 0, statusFan = 0, statusLed = 0, statusCover = 0, pullOut = 0;
/* VARIABLES SAVE UART */
char charR, null;
char stringR[18], arrData[20];
int id = -1;
/* VARIABLES SAVE RTC */
char rtc;
/* VARIABLES SAVE VALUE SET FOR MODE OF AUTO */
int lowerThreshold[3], upperThreshold[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DHT11_PORT GPIOA
#define DHT11_PIN DHT_Pin

void delayUs(uint16_t us);//delay micro second
void setPinIO(uint8_t io);//set input or output for DHT_PIN
void DHT11_Start ();//start communicate with DHT
uint8_t DHT11_Check_Response ();//check response from DHT
uint8_t DHT11_Read ();//get signal from DHT11
void lcdEnable();//set to enable send to LCD
void lcdSend4Bit(unsigned char data);//config 4 bit - LCD
void lcdCommand(unsigned char command);//send command
void lcdChar(unsigned char data);//send char
void lcdString(char *s);// send string
void lcdClear();//delete display
void lcdRowCol(unsigned char row, unsigned char collumn);//set row and collumn
void lcdInit();//Initnalize for LCD
void getValueDHT();//caculate and save DHT-value
void getValueSoilRainLight();//caculate and save SOIL - RAIN - LIGHT
void printValueSensor();//print value sensor to LCD
void controlDevice();//check and control device
void selectChannelADC(int selectMode);//select ADC-channel to use
void setValueControl();//control device according to the value setting  
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);//call to external interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);//call to UART interrupt
int convertToInt(char c);//convert char to int
char convertToChar(int i);//convert int to char

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char format1[18], format2[18];
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Base_Start(&htim2);
  lcdInit();
  lcdRowCol(1, 3);
  lcdString("SMART GARDEN");
  lcdRowCol(2, 2);
  lcdString("HUY-HIEU-TRONG");
  HAL_Delay(2000);
  lcdClear();
  lcdRowCol(1, 1);
  sprintf(format1, "T%c %cAIR%cSOIL%cAS", 0xDF, 0xFF, 0xFF, 0xFF);
  lcdString(format1);
  lcdRowCol(2, 3);
  sprintf(format2, "%c%c  %c%c  %c %c  %c", 0x43, 0xFF, 0x25, 0xFF, 0x25, 0xFF, 0x25);
  lcdString(format2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&charR,1);
  HAL_UART_Transmit(&huart1, (uint8_t*)&"Hello World!", 13, 100);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  getValueDHT();
    getValueSoilRainLight();
    printValueSensor();
    if(mode == '3')
      setValueControl();
    controlDevice();
		HAL_Delay(1200);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT_Pin|LCD_D4_Pin|LCD_EN_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|cover1_Pin
                          |cover2_Pin|LED_Pin|FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RAIN_Pin */
  GPIO_InitStruct.Pin = RAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RAIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT_Pin LCD_D4_Pin LCD_EN_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = DHT_Pin|LCD_D4_Pin|LCD_EN_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : pumpControl_Pin fanControl_Pin ledControl_Pin coverControl_Pin */
  GPIO_InitStruct.Pin = pumpControl_Pin|fanControl_Pin|ledControl_Pin|coverControl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin cover1_Pin
                           cover2_Pin LED_Pin FAN_Pin */
  GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|cover1_Pin
                          |cover2_Pin|LED_Pin|FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : switch1_Pin switch2_Pin */
  GPIO_InitStruct.Pin = switch1_Pin|switch2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void delayUs(uint16_t us){
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while(__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void setPinIO(uint8_t io){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT11_PIN;
  if(io){//set input
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
  }
  else{
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
  }
}
void DHT11_Start (void)
{
	setPinIO(0);  // set the pin as output
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1); //initialize with data pin high
	HAL_Delay(1); //wait for 1 milliseconds
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	delayUs (18000);   // wait for 18ms
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	delayUs (20);   // wait for 20us
	setPinIO(1);    // set as input
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	delayUs (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		delayUs (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		delayUs (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}
void lcdEnable(void){
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
}

void lcdSend4Bit(unsigned char data){
  HAL_GPIO_WritePin(GPIOA, LCD_D4_Pin, data & 0x01);
  HAL_GPIO_WritePin(GPIOB, LCD_D5_Pin, (data>>1) & 0x01);
  HAL_GPIO_WritePin(GPIOB, LCD_D6_Pin, (data>>2) & 0x01);
  HAL_GPIO_WritePin(GPIOB, LCD_D7_Pin, (data>>3) & 0x01);
}

void lcdCommand(unsigned char command){
  lcdSend4Bit(command >> 4);
  lcdEnable();
  lcdSend4Bit(command);
  lcdEnable();
}

void lcdChar(unsigned char data){
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin, GPIO_PIN_SET);
  lcdCommand(data);
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin, GPIO_PIN_RESET);
}

void lcdString(char *s){
  while(*s){
    lcdChar(*s);
    s++;
  }
}

void lcdClear(){
  lcdCommand(0x01);
  HAL_Delay(1);
}

void lcdRowCol(unsigned char row, unsigned char collumn){
  unsigned char address;
  if(row == 1)//hang 1
    address = 0x80 + (collumn - 1);
  else //hang 2
    address = 0xC0 + (collumn - 1);
  lcdCommand(address);
}
void lcdInit(){
  lcdSend4Bit(0x00);
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin, GPIO_PIN_RESET);
  lcdSend4Bit(0x03);
  lcdEnable();
  lcdEnable();
  lcdEnable();
  lcdSend4Bit(0x02);
  lcdEnable();
  lcdCommand(0x28);
  lcdCommand(0x0C);
  lcdCommand(0x06);
  lcdCommand(0x01);
}

void getValueDHT(){
  uint8_t Presence = 0, Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
  DHT11_Start();
  Presence = DHT11_Check_Response(); //record the response from the sensor
  if(Presence){
    Rh_byte1 = DHT11_Read ();//phan nguyen do am khong khi
    Rh_byte2 = DHT11_Read ();//phan du do am khong khi
    Temp_byte1 = DHT11_Read ();//phan nguyen nhiet do
    Temp_byte2 = DHT11_Read ();//phan du nhiet do
    SUM = DHT11_Read ();//phan tong kiem tra
    if(SUM == Rh_byte1 + Rh_byte2 + Temp_byte1  + Temp_byte2){
      Temperature =  Temp_byte1;
      Humidity =  Rh_byte1;
    }
  }
}

void getValueSoilRainLight(){
  selectChannelADC(8);
  HAL_ADC_Start(&hadc1);
  
  HAL_ADC_PollForConversion(&hadc1, 1000);
  Light = HAL_ADC_GetValue(&hadc1);
  Light = Light * 100 /4096;//anh sang
  HAL_ADC_Stop(&hadc1);
	HAL_Delay(100);
  selectChannelADC(0);
  HAL_ADC_Start(&hadc1);
  
  HAL_ADC_PollForConversion(&hadc1, 1000);
  Soil = HAL_ADC_GetValue(&hadc1);
  Soil = 100 - (Soil * 100 / 4096);//do am dat
  HAL_ADC_Stop(&hadc1);
	HAL_Delay(100);
  
  if(HAL_GPIO_ReadPin(GPIOC, RAIN_Pin))// trang thai rain sensor
    statusRain = '0';
  else
    statusRain = '1';
}

void printValueSensor(){
  lcdRowCol(2, 1);//dong 2 cot 1
  lcdChar(Temperature/10 + 0x30);
  lcdChar(Temperature%10 + 0x30);
  lcdRowCol(2, 5);//dong 2 cot 5
  lcdChar(Humidity/10 + 0x30);
  lcdChar(Humidity%10 + 0x30);
  lcdRowCol(2, 9);
  lcdChar(Soil/10 + 0x30);
  lcdChar(Soil%10 + 0x30);
  lcdRowCol(2, 14);
  lcdChar(Light/10 + 0x30);
  lcdChar(Light%10 + 0x30);;
}

void controlDevice(){

  if(statusPump == 1)
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 69);//"yeu"
  else if(statusPump == 2)
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 84);//"trung binh"
  else if(statusPump == 3)
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 99);//"manh"
  else
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, ~statusFan);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, statusLed);
  if(statusCover){//man che hoat dong
    if(pullOut == 0){//man che keo ra
      HAL_GPIO_WritePin(cover1_GPIO_Port, cover1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(cover2_GPIO_Port, cover2_Pin, GPIO_PIN_RESET);
      while(HAL_GPIO_ReadPin(switch1_GPIO_Port, switch1_Pin) == 0 && statusCover != 0);// doi den khi cham den cong tac hanh trinh 1 hoac nhan nut dung 
    }
    if(pullOut != 0){//man che keo vao
      HAL_GPIO_WritePin(cover1_GPIO_Port, cover1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(cover2_GPIO_Port, cover2_Pin, GPIO_PIN_SET);
      while(HAL_GPIO_ReadPin(switch2_GPIO_Port, switch2_Pin) == 0 && statusCover != 0);// doi den khi cham den cong tac hanh trinh 2 hoac nhan nut dung
    }
    if(statusCover != 0)
      pullOut = ~pullOut;// dao huong mai che o lan bat tiep theo
    statusCover = 0;
    HAL_GPIO_WritePin(cover1_GPIO_Port, cover1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cover2_GPIO_Port, cover2_Pin, GPIO_PIN_RESET);
    
  }
}

void selectChannelADC(int selectMode){
  ADC_ChannelConfTypeDef sConfig = {0};
  if(selectMode == 0)
    sConfig.Channel = ADC_CHANNEL_0;
  else
    sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
  
void setValueControl(){
//  if(statusRain == '0')
//    statusCover = pullOut != 0 ? ~statusCover : statusCover;//khong che
  if(Light > upperThreshold[0] && Temperature > upperThreshold[2]){
    statusCover = pullOut == 0 ? ~statusCover : statusCover;//che
    statusFan = statusFan == 0 ? ~statusFan : statusFan;//bat quat
  }
  if((Light < lowerThreshold[0] || Temperature < (upperThreshold[2] - 10)) && statusRain == '0'){
    statusCover = pullOut != 0 ? ~statusCover : statusCover;//khong che
    statusFan = statusFan != 0 ? ~statusFan : statusFan;//tat quat
  }
  if(statusRain == '1')
    statusCover = pullOut == 0 ? ~statusCover : statusCover;//che
  if(Soil < lowerThreshold[1] && statusPump == 0)
    statusPump = 3;
  if(Soil > upperThreshold[1] && statusPump != 0)
    statusPump = 0;
  if(Temperature < lowerThreshold[2])
    statusLed = statusLed == 0 ? ~statusLed : statusLed;
  if(Temperature > (lowerThreshold[2] + 5))
    statusLed = statusLed != 0 ? ~statusLed : statusLed;  
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == pumpControl_Pin){
    HAL_Delay(20);//chong rung phim
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0)
      statusPump = statusPump == 3 ? 0 : statusPump + 1;
  }
  else if (GPIO_Pin == fanControl_Pin){
    HAL_Delay(20);//chong rung phim
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
        statusFan = ~statusFan;
  }

  else if (GPIO_Pin == ledControl_Pin){
    HAL_Delay(20);//chong rung phim
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0)
        statusLed = ~statusLed;
  }

  else if (GPIO_Pin == coverControl_Pin){
    HAL_Delay(20);//chong rung phim
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
        statusCover = ~statusCover;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(charR != 13){
    stringR[++id] = charR;
  }
  if(charR == 13){
    if(stringR[id] == '!'){
      rtc = stringR[id];
    }
    else if(stringR[id] == '1')
      mode = '1';//che do dieu khien bang phim nhan
    else if(stringR[id] == '2'){
      mode = '2';//che do cho phep dieu khien tu xa
      statusPump = convertToInt(stringR[1]);
      if(stringR[2] != convertToChar(statusFan))
        statusFan = ~statusFan;
      if(stringR[3] != convertToChar(statusLed))
        statusLed = ~statusLed;
      if(stringR[4] != convertToChar(pullOut))
        statusCover = statusCover == 0 ? ~statusCover : statusCover;
    }
    else{
      mode = '3';//che do dieu khien tu dong
      lowerThreshold[0] = convertToInt(stringR[1]) * 10 + convertToInt(stringR[2]);//nguong anh sang duoi
      upperThreshold[0] = convertToInt(stringR[3]) * 10 + convertToInt(stringR[4]);//nguong anh sang tren
      lowerThreshold[1] = convertToInt(stringR[5]) * 10 + convertToInt(stringR[6]);//nguong do am dat duoi
      upperThreshold[1] = convertToInt(stringR[7]) * 10 + convertToInt(stringR[8]);//nguong do am dat tren
      lowerThreshold[2] = convertToInt(stringR[9]) * 10 + convertToInt(stringR[10]);//nguong nhiet do duoi
      upperThreshold[2] = convertToInt(stringR[11]) * 10 + convertToInt(stringR[12]);//nguong nhiet do tren    
    }
    sprintf(arrData, "%d%d%d%d%d%d%d%d%c/%d%c%c%c \n", Temperature/10, Temperature%10, Humidity/10, Humidity%10,
                                                  Soil/10, Soil%10, Light/10, Light%10, statusRain,
                                                  statusPump, convertToChar(statusFan), convertToChar(statusLed), convertToChar(pullOut));
    HAL_UART_Transmit(&huart1, (uint8_t*)&arrData, sizeof(arrData), 100);
    for(int count = 0; count < sizeof(stringR); count++)
      stringR[count] = NULL;
    id = -1;
  }
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&charR, 1);
}
int convertToInt(char c){
  return (int)c - 48;
}

char convertToChar(int i){
  if(i == 0)
    return '0';
  else
    return '1';
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
