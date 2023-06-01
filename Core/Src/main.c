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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "usbd_cdc_if.h"
#include "cell_measurement.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define kick_dog   IWDG->KR = 0xaaaa  // refresh watchdog;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t newline[] = {"\r\n"};
/*
static uint8_t buf[1000];
 uint8_t *half = &buf[0];
 uint8_t *full = &buf[499];
 uint16_t blocksize = sizeof(buf)/2;
 */
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  DBGMCU->CR = DBGMCU_CR_DBG_IWDG_STOP;  // Stop Watchdog in Debug Breakpoints

  uint8_t text[50] = {"\r\nStarting....\r\n"};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  kick_dog;
  usb_is_init();

  HAL_Delay(50);
  usb_vcp_send(text, sizeof(text));
  kick_dog;
  HAL_Delay(50);

  if(HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK){
	  memset(text,0,sizeof(text));
	  strcpy((char*)text,"ADC Calibration successful\r\n");
	  usb_vcp_send(text, sizeof(text));
  }else{
	  memset(text,0,sizeof(text));
	  strcpy((char*)text,"ADC Calibration failed\r\n");
	  usb_vcp_send(text, sizeof(text));
  }

  startMeasurement(&hadc1);
  while (1)
  {

	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	  ADC_STRUCT temp = {0};
	  for(uint8_t i = 0; i < RESULT_LENGTH; i++){
		  temp.ext_reference += result[i].ext_reference;
		  temp.voltage_cell_1 += result[i].voltage_cell_1;
		  temp.voltage_cell_2 += result[i].voltage_cell_2;
		  temp.voltage_cell_3 += result[i].voltage_cell_3;
		  temp.voltage_cell_4 += result[i].voltage_cell_4;
		  temp.current += result[i].current;
		  temp.temperature += result[i].temperature;
		  temp.int_reference += result[i].int_reference;
	  }
	  temp.ext_reference /= RESULT_LENGTH;
	  temp.voltage_cell_1 /= RESULT_LENGTH;
	  temp.voltage_cell_2 /= RESULT_LENGTH;
	  temp.voltage_cell_3 /= RESULT_LENGTH;
	  temp.voltage_cell_4 /= RESULT_LENGTH;
	  temp.current /= RESULT_LENGTH;
	  temp.temperature /= RESULT_LENGTH;
	  temp.int_reference /= RESULT_LENGTH;

	  kick_dog;
	  calculateAnalog();
	  memset(text,0,sizeof(text));
	  sprintf((char*)text,"%f",temp.temperature);
	  usb_vcp_send(text, sizeof(text));
	  strcpy((char*)text," Temp in C\r\n");
	  usb_vcp_send(text, sizeof(text));

	  memset(text,0,sizeof(text));
	  sprintf((char*)text,"%f",temp.int_reference);
	  usb_vcp_send(text, sizeof(text));
	  strcpy((char*)text," Ref in V\r\n");
	  usb_vcp_send(text, sizeof(text));



	  HAL_Delay(250);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
