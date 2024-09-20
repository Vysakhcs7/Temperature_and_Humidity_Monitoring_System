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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LCD_FUNCTION_SET1      0x33
#define LCD_FUNCTION_SET2      0x32
#define LCD_4BIT_2LINE_MODE    0x28
#define LCD_DISP_CURS_ON       0x0E
#define LCD_DISP_ON_CURS_OFF   0x0C  //Display on, cursor off
#define LCD_DISPLAY_OFF        0x08
#define LCD_DISPLAY_ON         0x04
#define LCD_CLEAR_DISPLAY      0x01
#define LCD_ENTRY_MODE_SET     0x04
#define LCD_INCREMENT_CURSER   0x06
#define LCD_SET_ROW1_COL1      0x80  //Force cursor to beginning ( 1st line)
#define LCD_SET_ROW2_COL1      0xC0  //Force cursor to beginning ( 2nd line)
#define LCD_MOVE_DISPLAY_LEFT  0x18
#define LCD_MOVE_DISPLAY_RIGHT 0x1C

#define slave_address      0x3F //LCD

#define AHT25_ADDRESS	(0x38 << 1U)
#define AHT25_RESET		(0xBA)
#define AHT25_INIT_CMD	(0x71)

#define DS1307_ADDRESS (0x68 << 1)  // Define DS1307 I2C address

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

uint8_t trigger_cmd[3] = { 0xAC, 0x33, 0x00 };
uint8_t status = 0;
uint8_t raw_reading[6];
float temperature, humidity;
uint32_t raw_humidity;
uint32_t raw_temperature;

char buffer_time[16];
char buffer_date[16];
uint8_t write_time_date[7];    // Buffer to store time data for writing
uint8_t read_time_date[7];    // Buffer to store time data read from DS1307
uint8_t bcd_time_date[7];    // Buffer to store time data in BCD format

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void LCD_Transmit_Command(uint8_t data);
void lcd_init(void);
void LCD_Transmit_Data(uint8_t data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void lcd_init(void) {

	/* Wait for 15ms */
	HAL_Delay(15);

	/*Function Set - As per HD44780U*/
	LCD_Transmit_Command(LCD_FUNCTION_SET1);

	/*Function Set -As per HD44780U*/
	LCD_Transmit_Command(LCD_FUNCTION_SET2);

	/*Set 4bit mode and 2 lines */
	LCD_Transmit_Command(LCD_4BIT_2LINE_MODE);

	/*Display on, cursor off*/
	LCD_Transmit_Command(0x0C);

	/* SET Row1 and Col1 (1st Line) */
	LCD_Transmit_Command(0x80);

	/*Clear Display*/
	LCD_Transmit_Command(LCD_CLEAR_DISPLAY);
}

void LCD_Transmit_Command(uint8_t data) {
	uint8_t command_en_on = ((data & 0xF0) | 0x0C); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t command_en_off = (command_en_on & (~(1 << 2))); //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_en_on, 1, 100);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_en_off, 1,
			100);

	uint8_t command_lsb_en_on = (((data << 4) & 0xF0) | 0X0C); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t command_lsb_en_off = (command_lsb_en_on & (~(1 << 2))); //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_lsb_en_on, 1,
			100);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_lsb_en_off, 1,
			100);
}

void LCD_Transmit_Data(uint8_t data) {
	uint8_t data_en_on = ((data & 0xF0) | 0x0D); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t data_en_off = (data_en_on & (~(1 << 2))); //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_en_on, 1, 100);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_en_off, 1, 100);

	uint8_t data_lsb_en_on = (((data << 4) & 0xF0) | 0X0D); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t data_lsb_en_off = (data_lsb_en_on & (~(1 << 2))); //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_lsb_en_on, 1,
			100);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_lsb_en_off, 1,
			100);
}

void LCD_Send_String(char *str) {
	while (*str) {
		LCD_Transmit_Data(*str++);
	}
}

// Convert decimal to BCD (Binary-Coded Decimal)
uint8_t DS1307_DEC2BCD(uint8_t dec) {
	return ((dec / 10) << 4) | (dec % 10);
}

// Convert BCD to decimal
uint8_t DS1307_BCD2DEC(uint8_t bcd) {
	return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Set the time on the DS1307 RTC
void DS1307_SetTime(uint8_t year, uint8_t month, uint8_t date, uint8_t day,
		uint8_t hour, uint8_t minute, uint8_t second) {
	write_time_date[0] = DS1307_DEC2BCD(second);  // Convert seconds to BCD
	write_time_date[1] = DS1307_DEC2BCD(minute);  // Convert minutes to BCD
	write_time_date[2] = DS1307_DEC2BCD(hour);    // Convert hours to BCD
	write_time_date[3] = DS1307_DEC2BCD(day);    // Convert hours to BCD
	write_time_date[4] = DS1307_DEC2BCD(date);    // Convert hours to BCD
	write_time_date[5] = DS1307_DEC2BCD(month);    // Convert hours to BCD
	write_time_date[6] = DS1307_DEC2BCD(year);    // Convert hours to BCD

	HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT,
			write_time_date, 7, HAL_MAX_DELAY);  // Write time data to DS1307
}

// Get the time from the DS1307 RTC
void DS1307_GetTime() {

	HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT,
			bcd_time_date, 7, HAL_MAX_DELAY);  // Read time data from DS1307
	for (int i = 0; i < 7; i++) {
		read_time_date[i] = DS1307_BCD2DEC(bcd_time_date[i]); // Convert BCD data to decimal
		sprintf(buffer_time, "T:%u:%u:%u", read_time_date[2], read_time_date[1],
				read_time_date[0]);  // Store the formatted string in 'buffer'
		sprintf(buffer_date, "D:%u-%u-%u", read_time_date[4], read_time_date[5],
				read_time_date[6]);  // Store the formatted string in 'buffer'
		LCD_Transmit_Command(LCD_SET_ROW1_COL1); // Set cursor at row 1, column 1
		LCD_Send_String(buffer_time);
		LCD_Transmit_Command(LCD_SET_ROW2_COL1); // Set cursor at row 1, column 1
		LCD_Send_String(buffer_date);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	char buffer_hum[16];  // Buffer to hold the converted string
	char buffer_temp[26];

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

	lcd_init();  // Initialize the LCD

	HAL_I2C_Master_Transmit(&hi2c1, AHT25_ADDRESS, (uint8_t*) AHT25_RESET, 1,
			100);
	HAL_Delay(100);
	HAL_I2C_Master_Transmit(&hi2c1, AHT25_ADDRESS, (uint8_t*) AHT25_INIT_CMD, 1,
			100);
	HAL_I2C_Master_Receive(&hi2c1, AHT25_ADDRESS, &status, 1, 100);
	if ((status & 0x18) == 0x18) {
		LCD_Transmit_Command(LCD_SET_ROW1_COL1); // Set cursor at row 1, column 1
		LCD_Send_String("Init Success");  // Display message
		HAL_Delay(500);

	} else {
		while (1) {
			LCD_Transmit_Command(LCD_CLEAR_DISPLAY);
			LCD_Transmit_Command(LCD_SET_ROW1_COL1); // Set cursor at row 1, column 1
			LCD_Send_String("Init Failed");  // Display message
			HAL_Delay(1000);
			LCD_Transmit_Command(LCD_CLEAR_DISPLAY);

		}
	}
	HAL_Delay(10);

	DS1307_SetTime(24, 9, 20, 5, 18, 24, 00); //YY-MM-DD-DAY-HR-MIN-SEC

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		LCD_Transmit_Command(LCD_CLEAR_DISPLAY);

		DS1307_GetTime();  // Get time

		LCD_Transmit_Command(LCD_CLEAR_DISPLAY);
		HAL_I2C_Master_Transmit(&hi2c1, AHT25_ADDRESS, trigger_cmd, 3, 100);

		HAL_Delay(80);
		HAL_I2C_Master_Receive(&hi2c1, AHT25_ADDRESS, &status, 1, 100);

		while ((status & (1 << 7))) {
			HAL_I2C_Master_Receive(&hi2c1, AHT25_ADDRESS, &status, 1, 100);
		}

		HAL_I2C_Master_Receive(&hi2c1, AHT25_ADDRESS, raw_reading, 6, 100);
		uint8_t temp_val;
		raw_humidity = raw_reading[1];
		raw_humidity = (raw_humidity << 8) | raw_reading[2];
		temp_val = raw_reading[3] & 0xF0;
		raw_humidity = (raw_humidity << 4) | (temp_val >> 4);

		temp_val = raw_reading[3] & 0x0F;
		raw_temperature = temp_val;
		raw_temperature = (raw_temperature << 8) | (raw_reading[4]);
		raw_temperature = (raw_temperature << 8) | (raw_reading[5]);

		humidity = (raw_humidity / 1048576.0) * 100.0;
		temperature = (((raw_temperature / 1048576.0) * 200.0) - 50.0);

		LCD_Transmit_Command(LCD_SET_ROW1_COL1); // Set cursor at row 1, column 1

		// Convert integer to string
		sprintf(buffer_hum, "Humidity: %.2f%%", humidity); // Store the formatted string in 'buffer'
		LCD_Send_String(buffer_hum);
		LCD_Transmit_Command(LCD_SET_ROW2_COL1); // Set cursor at row 1, column 1
		sprintf(buffer_temp, "Temp: %.2f C", temperature); // Store the formatted string in 'buffer'
		LCD_Send_String(buffer_temp);

		HAL_Delay(3000);

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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
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
