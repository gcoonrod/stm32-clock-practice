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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t counter = 0;
RTC_TimeTypeDef current_time;
SystemState sys_state = {
		.digits = {0},
		.dps = {0},
		.btn_set = 0,
		.btn_adj = 0
};

static const uint8_t segmentNumber[10] = { 0x3f, // 0
		0x06, // 1
		0x5b, // 2
		0x4f, // 3
		0x66, // 4
		0x6d, // 5
		0x7d, // 6
		0x07, // 7
		0x7f, // 8
		0x67  // 9
		};

void display_digit(uint8_t digit, uint8_t num) {
	// Turn off all segments
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port,
			LED_SEG_A_Pin | LED_SEG_B_Pin | LED_SEG_C_Pin | LED_SEG_D_Pin
					| LED_SEG_E_Pin | LED_SEG_F_Pin | LED_SEG_G_Pin
					| LED_SEG_DP_Pin, GPIO_PIN_RESET);

	// Turn off all digits
	HAL_GPIO_WritePin(LED_DIG_GPIO_Port,
			LED_DIG_1_Pin | LED_DIG_2_Pin | LED_DIG_3_Pin | LED_DIG_4_Pin
					| LED_DIG_5_Pin | LED_DIG_6_Pin, GPIO_PIN_SET);

	// Now, display the number
	//HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_A_Pin | LED_SEG_B_Pin | LED_SEG_C_Pin | LED_SEG_D_Pin | LED_SEG_E_Pin | LED_SEG_F_Pin | LED_SEG_G_Pin | LED_SEG_DP_Pin, segmentNumber[num]);
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_A_Pin,
			((segmentNumber[num] >> 0) & 0x01));
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_B_Pin,
			((segmentNumber[num] >> 1) & 0x01));
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_C_Pin,
			((segmentNumber[num] >> 2) & 0x01));
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_D_Pin,
			((segmentNumber[num] >> 3) & 0x01));
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_E_Pin,
			((segmentNumber[num] >> 4) & 0x01));
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_F_Pin,
			((segmentNumber[num] >> 5) & 0x01));
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_G_Pin,
			((segmentNumber[num] >> 6) & 0x01));
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_DP_Pin,
			((segmentNumber[num] >> 7) & 0x01));

	// Finally, turn on the desired digit
	HAL_GPIO_WritePin(LED_DIG_GPIO_Port, 1 << (digit + 10), GPIO_PIN_RESET);
}

void display_digit_dp(uint8_t digit, uint8_t num, uint8_t dp) {
	// Turn off all segments
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port,
				LED_SEG_A_Pin | LED_SEG_B_Pin | LED_SEG_C_Pin | LED_SEG_D_Pin
						| LED_SEG_E_Pin | LED_SEG_F_Pin | LED_SEG_G_Pin
						| LED_SEG_DP_Pin, GPIO_PIN_RESET);

	// Turn off all digits
	HAL_GPIO_WritePin(LED_DIG_GPIO_Port,
				LED_DIG_1_Pin | LED_DIG_2_Pin | LED_DIG_3_Pin | LED_DIG_4_Pin
						| LED_DIG_5_Pin | LED_DIG_6_Pin, GPIO_PIN_SET);

	uint8_t segments = (num < 10) ? segmentNumber[num] : 0;
	if (dp) segments |= (1 << 7);

	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_A_Pin,
				((segments >> 0) & 0x01));
		HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_B_Pin,
				((segments >> 1) & 0x01));
		HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_C_Pin,
				((segments >> 2) & 0x01));
		HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_D_Pin,
				((segments >> 3) & 0x01));
		HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_E_Pin,
				((segments >> 4) & 0x01));
		HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_F_Pin,
				((segments >> 5) & 0x01));
		HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_G_Pin,
				((segments >> 6) & 0x01));
		HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_DP_Pin,
				((segments >> 7) & 0x01));

		// Finally, turn on the desired digit
		HAL_GPIO_WritePin(LED_DIG_GPIO_Port, 1 << (digit + 10), GPIO_PIN_RESET);
}

void clear_display() {
	// Turn off all segments
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port,
			LED_SEG_A_Pin | LED_SEG_B_Pin | LED_SEG_C_Pin | LED_SEG_D_Pin
					| LED_SEG_E_Pin | LED_SEG_F_Pin | LED_SEG_G_Pin
					| LED_SEG_DP_Pin, GPIO_PIN_RESET);

	// Turn off all digits
	HAL_GPIO_WritePin(LED_DIG_GPIO_Port,
			LED_DIG_1_Pin | LED_DIG_2_Pin | LED_DIG_3_Pin | LED_DIG_4_Pin
					| LED_DIG_5_Pin | LED_DIG_6_Pin, GPIO_PIN_SET);
}

void displayNumber(uint32_t number) {
	if (number > 999999) {
		Error_Handler();
	}

	uint8_t digits[6] = { 0 };
	uint8_t i = 0;

	while (number > 0) {
		digits[i] = number % 10;
		number /= 10;
		i++;
	}

	for (uint8_t d = 0; d < 6; d++) {
		display_digit(d, digits[5 - d]);
		HAL_Delay(1);
		clear_display();
	}

}

void update_time(RTC_TimeTypeDef *time, SystemState *state) {

	state->digits[0] = (time->Hours & 0xF0) >> 4;
	state->digits[1] = time->Hours & 0x0F;
	state->digits[2] = (time->Minutes & 0xF0) >> 4;
	state->digits[3] = time->Minutes & 0x0F;
	state->digits[4] = (time->Seconds & 0xF0) >> 4;
	state->digits[5] = time->Seconds & 0x0F;

}

void check_buttons(SystemState* state) {
	state->btn_set = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_SET_Pin);
	state->btn_adj = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_ADJ_Pin);

	// TEMP
	state->dps[0] = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_SET_Pin) == GPIO_PIN_SET ? 0 : 1;
	state->dps[1] = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_ADJ_Pin) == GPIO_PIN_SET ? 0 : 1;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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

	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_D_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_E_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_F_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SEG_GPIO_Port, LED_SEG_DP_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_DIG_GPIO_Port,
			LED_DIG_1_Pin | LED_DIG_2_Pin | LED_DIG_3_Pin | LED_DIG_4_Pin
					| LED_DIG_5_Pin | LED_DIG_6_Pin, GPIO_PIN_SET);

	MX_RTC_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	sys_state.dps[0] = 0;
	sys_state.dps[1] = 1;
	sys_state.dps[2] = 0;
	sys_state.dps[3] = 1;
	sys_state.dps[4] = 0;
	sys_state.dps[5] = 1;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		// 1. Check for button presses
		check_buttons(&sys_state);

		// 2. Update the time
		HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BCD);
		update_time(&current_time, &sys_state);


		// 3. Redraw the display
		for (uint8_t d = 0; d < 6; d++) {
			display_digit_dp(d, sys_state.digits[d], sys_state.dps[d]);
			HAL_Delay(1);
			clear_display();
		}
		//displayNumber(counter++);
		//if (counter >= 999999) {
		//    counter = 0;
		//}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef DateToUpdate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
	DateToUpdate.Month = RTC_MONTH_NOVEMBER;
	DateToUpdate.Date = 0x4;
	DateToUpdate.Year = 0x23;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LED_SEG_A_Pin | LED_SEG_B_Pin | LED_SEG_C_Pin | LED_SEG_D_Pin
					| LED_SEG_E_Pin | LED_SEG_F_Pin | LED_SEG_G_Pin
					| LED_SEG_DP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			LED_DIG_1_Pin | LED_DIG_2_Pin | LED_DIG_3_Pin | LED_DIG_4_Pin
					| LED_DIG_5_Pin | LED_DIG_6_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_SEG_A_Pin LED_SEG_B_Pin LED_SEG_C_Pin LED_SEG_D_Pin
	 LED_SEG_E_Pin LED_SEG_F_Pin LED_SEG_G_Pin LED_SEG_DP_Pin */
	GPIO_InitStruct.Pin = LED_SEG_A_Pin | LED_SEG_B_Pin | LED_SEG_C_Pin
			| LED_SEG_D_Pin | LED_SEG_E_Pin | LED_SEG_F_Pin | LED_SEG_G_Pin
			| LED_SEG_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_DIG_1_Pin LED_DIG_2_Pin LED_DIG_3_Pin LED_DIG_4_Pin
	 LED_DIG_5_Pin LED_DIG_6_Pin */
	GPIO_InitStruct.Pin = LED_DIG_1_Pin | LED_DIG_2_Pin | LED_DIG_3_Pin
			| LED_DIG_4_Pin | LED_DIG_5_Pin | LED_DIG_6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	HAL_GPIO_WritePin(BTN_GPIO_Port, BTN_SET_Pin | BTN_ADJ_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = BTN_SET_Pin | BTN_ADJ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
