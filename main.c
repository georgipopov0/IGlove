/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define gesture_size 100

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_string_to_COMPORT(char *string);
void MPU6050_Init(void);
void MPU6050_Read_Accel (float *buff);
void record_gesture_raw(float gesture[gesture_size][3]);
void format_gesture(float gesture[gesture_size][3]);
void prototype_recognition(float gesture[gesture_size][3]);
void display_gesture_CMP(float gesture[gesture_size][3]);
void convert_acc_to_vel(float gesture[gesture_size][3]);
int find_biggest_ax(float vector3[3]);
void test_reading();
int read_button();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float gesture[gesture_size][3];
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(read_button()){
		  record_gesture_raw(gesture);
		  prototype_recognition(gesture);
	  }
  }

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void send_string_to_COMPORT(char *string)
{
	HAL_UART_Transmit(&huart2, string, strlen((char*)string), HAL_MAX_DELAY);
}

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void MPU6050_Read_Accel (float *buff)
{
	int16_t Accel_X_RAW = 0;
	int16_t Accel_Y_RAW = 0;
	int16_t Accel_Z_RAW = 0;

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	buff[0] = Accel_X_RAW/16384.0;
	buff[1] = Accel_Y_RAW/16384.0;
	buff[2] = Accel_Z_RAW/16384.0;
}


//  I'm not using the gyroscope, but i defined the read anyway.
//	Maybe i will need it later
void MPU6050_Read_Gyro (float *buff)
{
	int16_t Gyro_X_RAW = 0;
	int16_t Gyro_Y_RAW = 0;
	int16_t Gyro_Z_RAW = 0;

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	buff[1] = Gyro_X_RAW/131.0;
	buff[2] = Gyro_Y_RAW/131.0;
	buff[3] = Gyro_Z_RAW/131.0;
}

// Records a gesture
void record_gesture_raw(float gesture[gesture_size][3]){
	int i, j;
	float tmp[3];

	// reads the pure value
	for(i = 0; i < gesture_size; i++){
		MPU6050_Read_Accel(gesture[i]);
		HAL_Delay(50);
		MPU6050_Read_Accel(tmp);
		HAL_Delay(50);

		// for each record gets average between two values
		// this helps with the noise
		for(j = 0; j < 3; j++){
			gesture[i][j] = (tmp[j] + gesture[i][j]) / 2 ;
		}
		send_string_to_COMPORT("Recording\n\r");

		// checks if the button is still pressed
		if(read_button() == 0){
			break ;
		}
	}

	// sets the last read to 100
	// used to determine the count of read values
	gesture[i][0] = 100;
	gesture[i][1] = 100;
	gesture[i][2] = 100;
}

void format_gesture(float gesture[gesture_size][3]){
	int i,j;

	// Clear some of the noise
	for(i = 1; i < gesture_size; i++){
		for(j = 0; j < 3; j++){
			gesture[i][j] = (gesture[i - 1][j] + gesture[i][j]) / 2 ;
		}
	}

	gesture[0][0] = gesture[1][0];
	gesture[0][1] = gesture[1][1];
	gesture[0][2] = gesture[1][2];

	//remove initial acceleration
	for(i = 1; i < gesture_size; i++){
		for(j = 0; j < 3; j++){
			gesture[i][j] -= gesture[0][j];
		}
	}

	gesture[0][0] = 0;
	gesture[0][1] = 0;
	gesture[0][2] = 0;


	//round up to 3 digits after the ","
	for(i = 0; i < gesture_size; i++){
		for(j = 0; j < 3; j++){
			if( (gesture[i][j] < 0.01 && gesture[i][j] > 0)
				|| (gesture[i][j] > -0.01 && gesture[i][j] < 0)
			){
				gesture[i][j] = 0;
			}
		}
	}
}

// sends the contend of the gesture array to the comport
void display_gesture_CMP(float gesture[gesture_size][3]){
	char output_buff[20];

	for(int i = 0; i < gesture_size; i++){
			sprintf (output_buff, "X = %.2f ", gesture[i][0]);
			send_string_to_COMPORT(output_buff);

			sprintf (output_buff, "Y = %.2f ", gesture[i][1]);
			send_string_to_COMPORT(output_buff);

			sprintf (output_buff, "Z = %.2f\n\r", gesture[i][2]);
			send_string_to_COMPORT(output_buff);

			HAL_Delay(100);
	}
}

void test_reading(){
	char output_buff[20];
	float gesture[3];

	while(1){
			MPU6050_Read_Accel(gesture);

			sprintf (output_buff, "X = %.2f ", gesture[0]);
			send_string_to_COMPORT(output_buff);

			sprintf (output_buff, "Y = %.2f ", gesture[1]);
			send_string_to_COMPORT(output_buff);

			sprintf (output_buff, "Z = %.2f\n\r", gesture[2]);
			send_string_to_COMPORT(output_buff);

			HAL_Delay(200);
	}

}


//
void prototype_recognition(float gesture[gesture_size][3]){
	float avrg[3];
	char output_buff[20];
	int i,j;

	// Set the average 0
	for(j = 0;j < 3; j++){
		avrg[j] = 0;
	}

	// Calculate the average for all axis
	// and removes initial forces
	for(j = 0; j < 3; j++){
		for(i = 0; i < gesture_size && gesture[i][0] !=100; i++){
			avrg[j] += gesture[i][j];
		}
		avrg[j] /= i;
		avrg[j] -= gesture[0][j];
	}

	// Print the averages
	// debugging purposes
	sprintf (output_buff, "X = %.2f ", avrg[0]);
	send_string_to_COMPORT(output_buff);

	sprintf (output_buff, "Y = %.2f ", avrg[1]);
	send_string_to_COMPORT(output_buff);

	sprintf (output_buff, "Z = %.2f\n\r", avrg[2]);
	send_string_to_COMPORT(output_buff);

	// Actions to take on recognized gesture
	int index_of_biggest = find_biggest_ax(avrg);
	switch (index_of_biggest) {
		case 0:
			sprintf (output_buff, "Detected gesture: %s\n\r", "Left");
			break;
		case 1:
			sprintf (output_buff, "Detected gesture: %s\n\r", "Backward");
			break;
		case 2:
			sprintf (output_buff, "Detected gesture: %s\n\r", "Up");
			break;
		case 3:
			sprintf (output_buff, "Detected gesture: %s\n\r", "Right");
			break;
		case 4:
			sprintf (output_buff, "Detected gesture: %s\n\r", "Forward");
			break;
		case 5:
			sprintf (output_buff, "Detected gesture: %s\n\r", "Down");
			break;
		default:
			break;
	}

	send_string_to_COMPORT(output_buff);
}

int find_biggest_ax(float vector3[3]){
	float tmp[3];
	int largest_index = 0;

	// Makes the values positive
	for(int j = 0; j < 3; j++){
		if(vector3[j] < 0){
			tmp[j] = -1 * vector3[j];
		}else{
			tmp[j] = vector3[j];
		}
	}

	// Finds the index of the largest number
	for(int j = 0; j < 3; j++){
		if(tmp[j] > tmp[largest_index]){
			largest_index = j;
		}
	}

	// Separate positive and negative
	if(vector3[largest_index] < 0){
		largest_index += 3;
	}

	return largest_index;
}

// Convert the given gesture data form acceleration to velocity
void convert_acc_to_vel(float gesture[gesture_size][3]){
	int i,j;

	// set the initial value
	gesture[0][1] =  gesture[0][1]/0.025;


	//convert to velocity
	for(i = 1; i < gesture_size; i++){
		for(j = 1; j < 3; j++){
			gesture[i][j] = gesture[i][j]/0.025 + gesture[i-1][j];
		}
	}

}

int read_button(){
	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==GPIO_PIN_SET)
	  {
		  return 0;
	  }
	  else
	  {
		  return 1;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
