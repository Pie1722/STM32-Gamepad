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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "stm32f1xx_hal.h"
#include "NRF24.h"
#include "NRF24_conf.h"
#include "NRF24_reg_addresses.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct __attribute__((packed)) {
    uint16_t buttons;
    uint8_t hat;
    int16_t x, y, za, zr;
} joystickReport;


struct JoystickData {
  uint16_t code;
  uint16_t xAxis;
  uint16_t yAxis;
  uint16_t rXaxis;
  uint16_t rYaxis;
  uint16_t buttons;    // 16 bits, 16 buttons
};

struct JoystickData joy;

void reset_joystick_state(void)
{
    joy.code = 0;
    joy.xAxis = 0;
    joy.yAxis = 0;
    joy.rXaxis = 0;
    joy.rYaxis = 0;
    joy.buttons = 0;
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PLD_S sizeof(joy)

uint32_t led_timeout = 0;

//uint8_t tx_ack_pld[PLD_S];
uint8_t tx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21};

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

uint8_t get_hat_value(uint16_t btn)
{
	uint8_t up    = (btn >> 0) & 1;
	uint8_t down  = (btn >> 1) & 1;
	uint8_t left  = (btn >> 2) & 1;
	uint8_t right = (btn >> 3) & 1;

    if (up)            return 0;  // Up
    if (up && right)   return 1;  // Up-Right
    if (right)         return 2;  // Right
    if (right && down) return 3;  // Down-Right
    if (down)          return 4;  // Down
    if (down && left)  return 5;  // Down-Left
    if (left)          return 6;  // Left
    if (left && up)    return 7;  // Up-Left

    return 0x0F;  // Neutral
}


void send_gamepad_report(void)
{

	joystickReport report = {0};
    uint16_t btn = joy.buttons;

    // ---- Buttons: Read 10 GPIOs  ----
    report.buttons |= (((btn >> 9)  & 1) << 0);  // X
    report.buttons |= (((btn >> 10) & 1) << 1);  // A
    report.buttons |= (((btn >> 11) & 1) << 2);  // B
    report.buttons |= (((btn >> 8)  & 1) << 3);  // Y
    report.buttons |= (((btn >> 5)  & 1) << 4);  // LB
    report.buttons |= (((btn >> 7)  & 1) << 5);  // RB
    report.buttons |= (((btn >> 4)  & 1) << 6);  // LT
    report.buttons |= (((btn >> 6)  & 1) << 7);  // RT
    report.buttons |= (((btn >> 14) & 1) << 8);  // Back
    report.buttons |= (((btn >> 15) & 1) << 9);  // Start
    report.buttons |= (((btn >> 12) & 1) << 10); // L3
    report.buttons |= (((btn >> 13) & 1) << 11); // R3

    // ---- Hat switch ----
    report.hat = get_hat_value(btn);

    // ---- Axes: ADC static data ----

	report.x = joy.xAxis;
	report.y = joy.yAxis;
	report.zr = joy.rXaxis;
	report.za = joy.rYaxis;


    // ---- Send 11-byte report ----
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

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

  nrf24_listen();

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
  ce_high();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //reset_joystick_state();
	  nrf24_listen();

	  	if(nrf24_data_available()){
	  		nrf24_receive((uint8_t*)&joy, sizeof(joy));
	  		led_timeout = HAL_GetTick() + 100; // keep LED on 100ms
	  	}

	  	if (HAL_GetTick() < led_timeout) {
	  	    HAL_GPIO_WritePin(LED_BLTN_GPIO_Port, LED_BLTN_Pin, GPIO_PIN_RESET); // ON
	  	} else {
	  	    HAL_GPIO_WritePin(LED_BLTN_GPIO_Port, LED_BLTN_Pin, GPIO_PIN_SET);   // OFF
	  	}

	  send_gamepad_report();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CS_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(LED_BLTN_GPIO_Port, LED_BLTN_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
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
