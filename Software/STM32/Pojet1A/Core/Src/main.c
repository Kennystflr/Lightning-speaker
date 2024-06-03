/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "tim.h"
#include "ucpd.h"
#include "usart.h"
#include "usbpd.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "filter.h"
#include "sgtl5000.h"
#include "secboard.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SAI_TX_BUFFER_LENGTH (48*2)
#define SAI_RX_BUFFER_LENGTH (48*2)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t sai_tx_buffer[SAI_TX_BUFFER_LENGTH];
static int16_t sai_rx_buffer[SAI_RX_BUFFER_LENGTH];
int Encoder_AB[4] = {0,0,0,0};
int band_change = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int tx_cplt_counter = 0;
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	if (SAI1_Block_A == hsai->Instance)
	{
		tx_cplt_counter++;
		// TODO Temp juste pour voir
		//HAL_SAI_DMAStop(&hsai_BlockA2);
	}
}

static int tx_half_cplt_counter = 0;
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	if (SAI1_Block_A == hsai->Instance)
	{
		tx_half_cplt_counter++;
	}
}

static int rx_cplt_flag = 0;
static int rx_cplt_counter = 0;
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	if (SAI1_Block_B == hsai->Instance)
	{
		rx_cplt_flag = 1;
		rx_cplt_counter++;
	}
}

static int rx_half_cplt_flag = 0;
static int rx_half_cplt_counter = 0;
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	if (SAI1_Block_B == hsai->Instance)
	{
		rx_half_cplt_flag = 1;
		rx_half_cplt_counter++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == A1_Pin)
	{
		Encoder_AB[0] = 1-Encoder_AB[0];
	}
	if (GPIO_Pin == B1_Pin)
	{
		Encoder_AB[1] = 1-Encoder_AB[1];
	}
	if (GPIO_Pin == A2_Pin)
	{
		Encoder_AB[2] = 1-Encoder_AB[2];
	}
	if (GPIO_Pin == B2_Pin)
	{
		Encoder_AB[3] = 1-Encoder_AB[3];
	}
	if (GPIO_Pin == eq_band_btn_Pin)
	{
		band_change = 1;
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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_SAI1_Init();
	MX_UCPD1_Init();
	MX_ADC2_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART3_UART_Init();
	MX_USB_Device_Init();
	/* USER CODE BEGIN 2 */
	uint16_t value_pot;

	param_sgtl_t param_son;

	int freq;
	float gain;

	uint16_t sgtl_address = 0x14;
	uint16_t data;

	h_sgtl5000_t h_sgtl5000;
	h_sgtl5000.hi2c = &hi2c1;
	h_sgtl5000.dev_address = sgtl_address;

	sgtl5000_init(&h_sgtl5000);

	HAL_StatusTypeDef ret;
	ret = sgtl5000_i2c_read_register(&h_sgtl5000, SGTL5000_CHIP_ID, &data);


	if (ret != HAL_OK)
	{
		//printf("HAL_I2C_Mem_Read error\r\n");
		Error_Handler();
	}

	init_valeur_default(&param_son);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&value_pot,100);
	HAL_TIM_PWM_Init(&htim3);

	/* USER CODE END 2 */

	/* USBPD initialisation ---------------------------------*/
	MX_USBPD_Init();

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		USBPD_DPM_Run();

		/* USER CODE BEGIN 3 */
		modif_freq(&param_son,Encoder_AB[0],Encoder_AB[1]);
		modif_gain(&param_son,Encoder_AB[2],Encoder_AB[3]);

		volume(&param_son,&h_sgtl5000,value_pot);

		eq_band(&param_son,&freq,&gain);

		colormotion(&htim3,freq,gain);

		if(band_change == 1){
			ChangementEtat(&param_son,&h_sgtl5000);
			band_change = 0;
			(param_son.bandmod==3)?(param_son.bandmod=0):(param_son.bandmod++);
		}

		set_filter(&h_sgtl5000,freq,gain);
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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
		HAL_GPIO_WritePin(band0_GPIO_Port,band0_Pin,0);
		HAL_Delay(500);
		HAL_GPIO_WritePin(band1_GPIO_Port,band1_Pin,0);
		HAL_Delay(500);
		HAL_GPIO_WritePin(band2_GPIO_Port,band2_Pin,0);
		HAL_Delay(500);
		HAL_GPIO_WritePin(band3_GPIO_Port,band3_Pin,0);
		HAL_Delay(500);
		HAL_GPIO_WritePin(band0_GPIO_Port,band0_Pin,1);
		HAL_GPIO_WritePin(band1_GPIO_Port,band1_Pin,1);
		HAL_GPIO_WritePin(band2_GPIO_Port,band2_Pin,1);
		HAL_GPIO_WritePin(band3_GPIO_Port,band3_Pin,1);
		HAL_Delay(500);
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
