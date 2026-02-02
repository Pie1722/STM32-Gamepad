/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "NRF24.h"
#include "NRF24_conf.h"
#include "NRF24_reg_addresses.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

struct JoystickData {
  uint16_t code;
  uint16_t xAxis;
  uint16_t yAxis;
  uint16_t rXaxis;
  uint16_t rYaxis;
  uint16_t buttons;    // 16 bits, 16 buttons
};

struct JoystickData joy;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PLD_S sizeof(joy)

//uint8_t tx_ack_pld[PLD_S];
uint8_t tx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21};

volatile uint16_t adcVal[4] = {0};
volatile int8_t adcValueReady;
uint32_t x,y,lx,ly;
uint16_t xRightStickValue_ADC, yRightStickValue_ADC, yLeftStickValue_ADC, xLeftStickValue_ADC;
uint16_t xRightCenter, yRightCenter, xLeftCenter, yLeftCenter;
volatile int8_t count;

//uint8_t msg[4] = {'P','I','E','!'};

//uint8_t dataT[PLD_S];

// Simple NRF24 connection check
// Returns 1 if OK, 0 if FAIL
uint8_t nrf24_is_connected(void)
{
    uint8_t testVal = 0x0A;  // Random check pattern
    uint8_t readVal = 0;

    // Write CONFIG register
    nrf24_w_reg(CONFIG, &testVal, 1);

    // Read CONFIG register
    readVal = nrf24_r_reg(CONFIG, 1);

    return (readVal == testVal);
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int abs(int x)
{
    if (x < 0)
        return -x;
    else
        return x;
}

void sendButtons() {


	  joy.buttons = 0;  // reset all 16 bits

	  // D-PAD
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)  << 0);  // UP
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)  << 1);  // DOWN
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) << 2);  // LEFT
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  << 3);  // RIGHT

	  // TRIGGERS
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  << 4);  // LT1
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  << 5);  // LT2
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) << 6);  // RT1
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)  << 7);  // RT2

	  // FACE & MISC
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) << 8);   // Y
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) << 9);   // X
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) << 10);  // A
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) << 11);  // B
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)  << 12);  // L3
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)  << 13);  // R3
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)  << 14);  // BACK
	  joy.buttons |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)  << 15);  // START

	  int16_t drx = xRightStickValue_ADC - xRightCenter;
	  int16_t dry = yRightStickValue_ADC - yRightCenter;
	  int16_t dlx = xLeftStickValue_ADC  - xLeftCenter;
	  int16_t dly = yLeftStickValue_ADC  - yLeftCenter;

	  // dead-zone (kill noise around center)
	  if (abs(drx) < 50) drx = 0;
	  if (abs(dry) < 50) dry = 0;
	  if (abs(dlx) < 50) dlx = 0;
	  if (abs(dly) < 50) dly = 0;

	  // end-zones (clip to safe ADC span)
	  if (drx >  2047) drx =  2047;
	  if (drx < -2047) drx = -2047;

	  if (dry >  2047) dry =  2047;
	  if (dry < -2047) dry = -2047;

	  if (dlx >  2047) dlx =  2047;
	  if (dlx < -2047) dlx = -2047;

	  if (dly >  2047) dly =  2047;
	  if (dly < -2047) dly = -2047;

	  joy.xAxis  = map(drx, -2047, 2047, -32767, 32767);
	  joy.yAxis  = map(dry, -2047, 2047, -32767, 32767);
	  joy.rXaxis = map(dlx, -2047, 2047, -32767, 32767);
	  joy.rYaxis = map(dly, -2047, 2047, -32767, 32767);


	  joy.code = 457;

	 // check = nrf24_transmit((uint8_t*)&joy, sizeof(joy));

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcVal, 4);

  csn_high();
  ce_high();

  HAL_Delay(5);

  ce_low();

  nrf24_init();

  HAL_Delay(5);

  if (!nrf24_is_connected()) {
      // Blink forever → NRF missing or SPI problem
      while (1) {
          HAL_GPIO_TogglePin(LED_BLTN_GPIO_Port, LED_BLTN_Pin);
          HAL_Delay(200);
      }
  }

  nrf24_stop_listen();

  nrf24_auto_ack_all(auto_ack);
  nrf24_en_ack_pld(disable);
  nrf24_dpl(disable);

  nrf24_set_crc(en_crc, _2byte);

  nrf24_tx_pwr(_0dbm);
  nrf24_data_rate(_2mbps);
  nrf24_set_channel(76);
  nrf24_set_addr_width(5);

  nrf24_set_rx_dpl(0, disable);
  nrf24_set_rx_dpl(1, disable);
  nrf24_set_rx_dpl(2, disable);
  nrf24_set_rx_dpl(3, disable);
  nrf24_set_rx_dpl(4, disable);
  nrf24_set_rx_dpl(5, disable);

  nrf24_pipe_pld_size(0, PLD_S);

  nrf24_auto_retr_delay(4);
  nrf24_auto_retr_limit(10);

  nrf24_open_tx_pipe(tx_addr);
  nrf24_open_rx_pipe(0, tx_addr);

  //ce_high();

  while (!adcValueReady);   // wait for first averaged ADC frame

  xRightCenter = xRightStickValue_ADC;
  yRightCenter = yRightStickValue_ADC;
  xLeftCenter = xLeftStickValue_ADC;
  yLeftCenter = yLeftStickValue_ADC;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  sendButtons();
	  uint8_t check = nrf24_transmit((uint8_t*)&joy, sizeof(joy));
//	  if (check == 1) {
//	      HAL_GPIO_WritePin(LED_BLTN_GPIO_Port, LED_BLTN_Pin, GPIO_PIN_SET);  // SUCCESS
//	  } else {
//	      HAL_GPIO_WritePin(LED_BLTN_GPIO_Port, LED_BLTN_Pin, GPIO_PIN_RESET); // FAIL
//	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLTN_GPIO_Port, LED_BLTN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BLTN_Pin */
  GPIO_InitStruct.Pin = LED_BLTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 PA9 PA10
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CS_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB3 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**	ADC ISR
*		Handles the values from ADC after the conversion finished
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

    x  += adcVal[2];
    y  += adcVal[3];
    lx += adcVal[0];
    ly += adcVal[1];

    count++;

    // after 100 samples, average them
    if (count >= 100) {

        xRightStickValue_ADC = x / 100;
        yRightStickValue_ADC = y / 100;
        xLeftStickValue_ADC  = lx / 100;
        yLeftStickValue_ADC  = ly / 100;

        // reset for next round
        x = 0;
        y = 0;
        lx = 0;
        ly = 0;
        count = 0;

        adcValueReady = 1;
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
	  HAL_GPIO_TogglePin(LED_BLTN_GPIO_Port, LED_BLTN_Pin);
	  HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
