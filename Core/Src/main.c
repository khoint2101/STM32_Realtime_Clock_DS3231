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
//#include "CLCD_I2C.h"
#include "i2c-lcd.h"
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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
#define DS3231_ADDRESS		(0x68<<1)     // I2C address; dich bit de lay dia chi chuan khung truyen
#define DS3231_REG_TIME    0x00					// thanh ghi dau tien cua ds3231 vs dia chi 0x00

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;	
	
	uint8_t I2C_Buffer[8];
}DS3231_t;

DS3231_t DS3231;


//CLCD_I2C_Name LCD1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t RTC_BCD2DEC(uint8_t c);
uint8_t RTC_DEC2BCD(uint8_t c);
void RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t date, uint8_t day, uint8_t month, uint8_t year);
void RTC_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *date, uint8_t *day, uint8_t *month, uint8_t *year);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RTC_BCD2DEC(uint8_t c)  // chuyen doi BCD to DEC de doc data tu DS3231
{ 
	return (c>>4)*10 + (c&0x0f);

}
uint8_t RTC_DEC2BCD(uint8_t c)  // chuyen doi DEC to BCD de nap gia tri cho DS3231
{
	return (c/10)<<4|(c%10);
}


void RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t date ,uint8_t day, uint8_t month, uint8_t year)
{					
	uint8_t Buf[7] = {RTC_DEC2BCD(sec),RTC_DEC2BCD(min),RTC_DEC2BCD(hour),RTC_DEC2BCD(date),
	RTC_DEC2BCD(day),RTC_DEC2BCD(month),RTC_DEC2BCD(year)};
	HAL_I2C_Mem_Write(&hi2c1,DS3231_ADDRESS,0x00,I2C_MEMADD_SIZE_8BIT,Buf,7,1000);
	
}

void RTC_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *date, uint8_t *day, uint8_t *month, uint8_t *year)
{
	uint8_t Buf[8];
	HAL_I2C_Mem_Read(&hi2c1,DS3231_ADDRESS,0x00,I2C_MEMADD_SIZE_8BIT,Buf,7,100);
	*hour = RTC_BCD2DEC(Buf[2]);
	*min = RTC_BCD2DEC(Buf[1]);
	*sec = RTC_BCD2DEC(Buf[0]);
	*date = RTC_BCD2DEC(Buf[3]);
	*day = RTC_BCD2DEC(Buf[4]);
	*month = RTC_BCD2DEC(Buf[5]);
	*year = RTC_BCD2DEC(Buf[6]);
	
}

	char arr_str[20];
  char arr_str1[20];
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	//RTC_SetTime(12,30,0,25,2,23);
//	CLCD_I2C_Init(&LCD1, &hi2c1, 0x4e, 16,2);
//	CLCD_I2C_SetCursor(&LCD1, 0, 0);
//	CLCD_I2C_WriteString(&LCD1,"RTC CLOCK");
	lcd_init();
	lcd_goto_XY(1,0);
	lcd_send_string("Realtime Clock");
	lcd_goto_XY(2,0);
	lcd_send_string("Nhom 10");
	//RTC_SetTime(23,13,30,1,13,3,23);   // Gio/phut/giay/Ngay/Thu/Thang/Nam
	//rtc_write_time(22,35,12);
	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//rtc_read_time(&a,&b,&c);
		RTC_GetTime(&DS3231.hour,&DS3231.min,&DS3231.sec,&DS3231.date,&DS3231.day,&DS3231.month,&DS3231.year);
		lcd_clear_display();
		lcd_goto_XY(1,0);
		sprintf(arr_str,"      %02d:%02d:%02d",DS3231.hour,DS3231.min,DS3231.sec);
		lcd_send_string(arr_str);
		lcd_goto_XY(2,3);
		sprintf(arr_str1,"%02d/%02d/%02d",DS3231.day,DS3231.month,DS3231.year +2000);
		lcd_send_string(arr_str1);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
