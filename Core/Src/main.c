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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_CHANNEL_MIN      990
#define RC_CHANNEL_MAX      2010
#define RC_RAW_MIN          1000
#define RC_RAW_MAX          2000
#define LED_TIME            300
#define SBUS_MIN_OFFSET     173
#define SBUS_MID_OFFSET     992
#define SBUS_MAX_OFFSET     1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH  25
#define SBUS_FRAME_HEADER   0x0f
#define SBUS_FRAME_FOOTER   0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE  0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE    15   // ms
#define ADC_DEADBAND        40
#define ADC_CENTRE          2048
#define DISPLAY_UPDATE_RATE 100  // ms

// OLED display layout
#define SLIDER_X_START      28   // pixels from left (after label)
#define SLIDER_X_END        127  // pixels from left (right edge)
#define SLIDER_WIDTH        (SLIDER_X_END - SLIDER_X_START)
#define SLIDER_HEIGHT       8    // pixels tall
#define SLIDER_STEER_Y      26
#define SLIDER_THROTTLE_Y   42
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t adcDmaBuffer[2];  // [0] = CH4 throttle (PA4), [1] = CH8 steer (PB0)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Map integer from one range to another
static inline int map_int(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Apply centre deadband to raw ADC value
static inline int applyDeadband(uint32_t raw) {
    if (raw > ADC_CENTRE - ADC_DEADBAND && raw < ADC_CENTRE + ADC_DEADBAND)
        return ADC_CENTRE;
    return (int)raw;
}

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe) {

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 1000-2000 with middle at 1500 channel values to
     * 173-1811 with middle at 992 as S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map_int(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) stateByte |= SBUS_STATE_SIGNALLOSS;
    if (isFailsafe)   stateByte |= SBUS_STATE_FAILSAFE;

    packet[0]  = SBUS_FRAME_HEADER;
    packet[1]  = (uint8_t) (output[0] & 0x07FF);
    packet[2]  = (uint8_t) ((output[0] & 0x07FF)>>8  | (output[1] & 0x07FF)<<3);
    packet[3]  = (uint8_t) ((output[1] & 0x07FF)>>5  | (output[2] & 0x07FF)<<6);
    packet[4]  = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5]  = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6]  = (uint8_t) ((output[3] & 0x07FF)>>7  | (output[4] & 0x07FF)<<4);
    packet[7]  = (uint8_t) ((output[4] & 0x07FF)>>4  | (output[5] & 0x07FF)<<7);
    packet[8]  = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9]  = (uint8_t) ((output[5] & 0x07FF)>>9  | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6  | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8  | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5  | (output[10] & 0x07FF)<<6);
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7  | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4  | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9  | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6  | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);
    packet[23] = stateByte;
    packet[24] = SBUS_FRAME_FOOTER;
}

// Returns 1950 (armed) or 1150 (disarmed)
int getArm(void) {
    return (!HAL_GPIO_ReadPin(ARM_SWITCH_GPIO_Port, ARM_SWITCH_Pin)) ? 1950 : 1150;
}

// Returns RC value for flight mode: Hold=1165, Manual=1425, Auto=1815
int getMode(void) {
    if (!HAL_GPIO_ReadPin(FLIGHT_SWITCH_A_GPIO_Port, FLIGHT_SWITCH_A_Pin))
        return 1815;  // Auto
    else if (!HAL_GPIO_ReadPin(FLIGHT_SWITCH_B_GPIO_Port, FLIGHT_SWITCH_B_Pin))
        return 1165;  // Hold
    else
        return 1425;  // Manual (default/middle)
}

// Draw a horizontal slider bar on the OLED
// rc_val: 1000-2000, y_pos: top pixel of the bar
static void drawSlider(int rc_val, uint8_t y_pos) {
    // Outer border
    ssd1306_DrawRectangle(SLIDER_X_START, y_pos, SLIDER_X_END, y_pos + SLIDER_HEIGHT, White);

    // Thumb position: map rc_val 1000-2000 to pixel range
    int thumb_x = map_int(rc_val, RC_RAW_MIN, RC_RAW_MAX, SLIDER_X_START + 1, SLIDER_X_END - 4);

    // Filled thumb (4px wide)
    ssd1306_FillRectangle(thumb_x, y_pos + 1, thumb_x + 3, y_pos + SLIDER_HEIGHT - 1, White);
}

// Update the OLED with current rc channel values
static void updateDisplay(int steer_rc, int throttle_rc, int arm_val, int mode_val) {
    ssd1306_Fill(Black);

    // --- Top row: arm state and mode label ---
    const char *arm_str  = (arm_val  > 1500) ? "ARMED" : "DISARM";
    const char *mode_str;
    if      (mode_val > 1600) mode_str = "GUIDED";
    else if (mode_val < 1300) mode_str = "MANUAL";
    else                      mode_str = "HOLD";

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(arm_str, Font_7x10, White);
    ssd1306_SetCursor(80, 0);
    ssd1306_WriteString(mode_str, Font_7x10, White);

    // --- Steer slider ---
    ssd1306_SetCursor(0, SLIDER_STEER_Y);
    ssd1306_WriteString("STR", Font_7x10, White);
    drawSlider(steer_rc, SLIDER_STEER_Y);

    // --- Throttle slider ---
    ssd1306_SetCursor(0, SLIDER_THROTTLE_Y);
    ssd1306_WriteString("THR", Font_7x10, White);
    drawSlider(throttle_rc, SLIDER_THROTTLE_Y);

    ssd1306_UpdateScreen();
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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(200);
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  // Start ADC in DMA circular mode — buffer updates continuously
  HAL_ADC_Start_DMA(&hadc, adcDmaBuffer, 2);
  __HAL_DMA_DISABLE_IT(&hdma_adc, DMA_IT_TC | DMA_IT_HT);

  // Init OLED
  HAL_Delay(500);  // let display power up
  ssd1306_Init();

  uint8_t  sbusPacket[SBUS_PACKET_LENGTH];
  int      rcChannels[SBUS_CHANNEL_NUMBER];
  uint32_t sbusTime    = 0;
  uint32_t ledTime     = 0;
  uint32_t displayTime = 0;
  uint8_t  ledState    = 0;

  for (int i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
      rcChannels[i] = 1500;  // safe mid value
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t now = HAL_GetTick();

    // --- SBUS transmit at 15ms ---
    if (now - sbusTime >= SBUS_UPDATE_RATE) {

        uint32_t steer    = applyDeadband(adcDmaBuffer[1]);  // CH8 PB0
        uint32_t throttle = applyDeadband(adcDmaBuffer[0]);  // CH4 PA4

        rcChannels[0] = map_int(steer,    0, 4095, RC_RAW_MIN, RC_RAW_MAX);
        rcChannels[2] = map_int(throttle, 0, 4095, RC_RAW_MIN, RC_RAW_MAX);
        rcChannels[5] = getArm();
        rcChannels[7] = getMode();

        sbusPreparePacket(sbusPacket, rcChannels, false, false);
        HAL_UART_Transmit(&huart2, sbusPacket, SBUS_PACKET_LENGTH, 5);

        sbusTime = now;
    }

    // --- OLED update at 100ms ---
    if (now - displayTime >= DISPLAY_UPDATE_RATE) {
        updateDisplay(rcChannels[0], rcChannels[2], rcChannels[5], rcChannels[7]);
        displayTime = now;
    }

    // --- Status LED toggle at 300ms ---
    if (now - ledTime > LED_TIME) {
        ledState = !ledState;
        HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin,
                          ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        ledTime = now;
    }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPARE_IO_1_Pin SPARE_IO_2_Pin ARM_SWITCH_Pin FLIGHT_SWITCH_A_Pin
                           FLIGHT_SWITCH_B_ACT_Pin */
  GPIO_InitStruct.Pin = SPARE_IO_1_Pin|SPARE_IO_2_Pin|ARM_SWITCH_Pin|FLIGHT_SWITCH_A_Pin
                          |FLIGHT_SWITCH_B_ACT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLIGHT_SWITCH_B_Pin */
  GPIO_InitStruct.Pin = FLIGHT_SWITCH_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLIGHT_SWITCH_B_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
      for (volatile int i = 0; i < 80000; i++);  // ~100ms at 8MHz HSI
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
