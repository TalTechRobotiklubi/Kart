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

#include <cmsis_gcc.h>
#include <gpio.h>
#include <i2c.h>
#include <lis2dw12_reg.h>
#include <stdio.h>
#include <stm32f3xx_hal_def.h>
#include <stm32f3xx_hal_flash.h>
#include <stm32f3xx_hal_i2c.h>
#include <stm32f3xx_hal_rcc.h>
#include <stm32f3xx_hal_rcc_ex.h>
#include <stm32f3xx_hal_uart.h>
#include <sys/_stdint.h>
#include <usart.h>
#include "main.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t i2cWrite_Accel(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t i2cRead_Accel(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t i2cWrite_Gyro( uint8_t reg, const uint8_t *bufp);
int32_t i2cRead_Gyro( int8_t reg,  uint8_t *bufp, uint16_t len);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	stmdev_ctx_t device_interface;
	device_interface.write_reg = i2cWrite_Accel;
	device_interface.read_reg = i2cRead_Accel;
	device_interface.mdelay = HAL_Delay;
	device_interface.handle = &hi2c1;

	static uint8_t tx_buffer[1000];
	static int16_t data_raw_acceleration[3];
	static int16_t data_raw_rotationAccel[3];
	static float acceleration_mg[3];
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    uint8_t whoamI, rst;
	lis2dw12_device_id_get(&device_interface, &whoamI);
	if (whoamI != LIS2DW12_ID)
	{
	  while(1);
	}
	lis2dw12_reset_set(&device_interface, PROPERTY_ENABLE);
	do
	{
	  lis2dw12_reset_get(&device_interface, &rst);
	} while(rst);

	 /* Enable Block Data Update */
	  lis2dw12_block_data_update_set(&device_interface, PROPERTY_ENABLE);
	  /* Set full scale */
	  lis2dw12_full_scale_set(&device_interface, LIS2DW12_2g);
	  /* Configure filtering chain
	   * Accelerometer - filter path / bandwidth
	   */
	  lis2dw12_filter_path_set(&device_interface, LIS2DW12_LPF_ON_OUT);
	  lis2dw12_filter_bandwidth_set(&device_interface, LIS2DW12_ODR_DIV_4);
	  /* Configure power mode */
	  lis2dw12_power_mode_set(&device_interface, LIS2DW12_HIGH_PERFORMANCE);
	  /* Set Output Data Rate */
	  lis2dw12_data_rate_set(&device_interface, LIS2DW12_XL_ODR_25Hz);


	  i2cRead_Gyro(0x0F,  &whoamI, 1);
	  	if (whoamI != 0x68)
	  	{
	  	  while(1);
	  	}
	  	// THIS IS NOT THE PROPER WAY TO THIS. SHOULD ONLY CHANGE CERTAIN BITS IN REGISTRY. function maybe fixable
	  	i2cWrite_Gyro( 0x20, 0b00001111);//emables device and axis
	  	i2cWrite_Gyro(0x21, 0b00000000); // highest cutoff frec, assentially sisables lowpass
	  	i2cWrite_Gyro(0x23, 0b00000000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t reg;
	  uint8_t rotAccReady;
	      /* Read output only if new value is available */
	      lis2dw12_flag_data_ready_get(&device_interface, &reg);

	      if (reg) {
	        /* Read acceleration data */
	        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	        lis2dw12_acceleration_raw_get(&device_interface, data_raw_acceleration);
	        acceleration_mg[0] = lis2dw12_from_fs2_to_mg(
	                               data_raw_acceleration[0]);
	        acceleration_mg[1] = lis2dw12_from_fs2_to_mg(
	                               data_raw_acceleration[1]);
	        acceleration_mg[2] = lis2dw12_from_fs2_to_mg(
	                               data_raw_acceleration[2]);
	        sprintf((char *)tx_buffer,"Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	        HAL_UART_Transmit(&huart2, tx_buffer, strlen((char const *)tx_buffer), 200);
	      }

	      i2cRead_Gyro(0x27,&rotAccReady);
	      if(rotAccReady & 0b1000)
	      {
	    	  memset(data_raw_rotationAccel, 0x00, 3 * sizeof(int16_t));

	      }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//wrapers for accelerometer driver
int32_t i2cWrite_Accel(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

int32_t i2cWrite_Gyro( uint8_t reg, const uint8_t *bufp)
{
	//TODO: read the registry, make only necessary bit changes (bitwise or/and ?), send registry
	HAL_I2C_Mem_Write(&hi2c1, 0x68, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, 1, 1000);
	return 0;
}

int32_t i2cRead_Accel(void *handle, uint8_t reg,  uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

int32_t i2cRead_Gyro( int8_t reg,  uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(&hi2c1, 0x68, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
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
