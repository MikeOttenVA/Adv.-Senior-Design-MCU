/* USER CODE BEGIN Header */
/*
	Author: Michael Ottenberg
	Date: 04.11.2024

	Created using the ADC1256, NUCLEO F439ZI, and a Current Transformer to:
	read current waveform from ADC, calculate THD data, and send to a virtual serial port

	With Reference From: Jure Bartol
	Date: 07.05.2016

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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ads1256.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
//#include "usbd_cdc_if.h"
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
// global variables for SPI communication

// Send 8 bit value over serial interface (SPI).
// Changed to use HAL, CS pin on B6
void send8bit(uint8_t data)
{
	HAL_SPI_Transmit(&hspi3, (uint8_t *)&(data), 1, 1);
}

// Recieve 8 bit value over serial interface (SPI).
uint8_t recieve8bit(void)
{
	uint8_t data = 0;
	HAL_SPI_Receive(&hspi3, (uint8_t *)&data, 1, 1);
	return data;
}

// Read 1 byte from register address registerID.
// This could be modified to read any number of bytes from register!
uint8_t readByteFromReg(uint8_t registerID)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	waitDRDY();
	send8bit(CMD_RREG | registerID); // 1st byte: address of the first register to read
	send8bit(0x00); 				 // 2nd byte: number of bytes to read = 1.

//	delayus(10); 	// min delay: t6 = 50 * 1/freq.clkin = 50 * 1 / 7,68 Mhz = 6.5 micro sec
	uint8_t read = recieve8bit();
//	HAL_SPI_Receive(&hspi1, (uint8_t *)&data, 1, 100);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	return read;
}

// Write value (1 byte) to register address registerID.
// This could be modified to write any number of bytes to register!
void writeByteToReg(uint8_t registerID, uint8_t value)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	send8bit(CMD_WREG | registerID); // 1st byte: address of the first register to write
	send8bit(0x00); 				 // 2nd byte: number of bytes to write = 1.
	send8bit(value);				 // 3rd byte: value to write to register
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

// Send standalone commands to register.
void writeCMD(uint8_t command)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	send8bit(command);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

// Wait until DRDY is low.
void waitDRDY(void)
{
	GPIO_PinState dRdyPinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	while(dRdyPinState == GPIO_PIN_RESET){
		dRdyPinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
		continue;
	}
}

// Write to A/D data rate register - set data rate.
void setDataRate(uint8_t drate)
{
	writeByteToReg(REG_DRATE, drate);
}

// Set the internal buffer (True - enable, False - disable).
void setBuffer(bool val)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	send8bit(CMD_WREG | REG_STATUS);
	send8bit((0 << 3) | (1 << 2) | (val << 1));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

// Get data from STATUS register - chip ID information.
uint8_t readChipID(void)
{
	waitDRDY();
	uint8_t id = readByteFromReg(REG_STATUS);
	return (id); // Only bits 7,6,5,4 are the ones to read (only in REG_STATUS) - return shifted value!
}

// Write to MUX register - set channel to read from in single-ended mode.
// Bits 7,6,5,4 determine the positive input channel (AINp).
// Bits 3,2,1,0 determine the negative input channel (AINn).
void setSEChannel(uint8_t channel)
{
	writeByteToReg(REG_MUX, channel << 4 | 1 << 3); // xxxx1000 - AINp = channel, AINn = AINCOM
}

// Write to MUX register - set channel to read from in differential mode.
// Bits 7,6,5,4 determine the positive input channel (AINp).
// Bits 3,2,1,0 determine the negative input channel (AINn).
void setDIFFChannel(uint8_t positiveCh, uint8_t negativeCh)
{
	writeByteToReg(REG_MUX, positiveCh << 4 | negativeCh); // xxxx1000 - AINp = positiveCh, AINn = negativeCh
}

// Write to A/D control register - set programmable gain amplifier (PGA).
// CLKOUT and sensor detect options are turned off in this case.
void setPGA(uint8_t pga)
{
	writeByteToReg(REG_ADCON, pga); // 00000xxx, Note: xxx = pga
}

// Continuously acquire analog data from one single-ended analog input.
// Allows sampling of one single-ended input channel up to 30,000 SPS.
void scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, uint32_t *values)
{
	uint8_t buffer[3];
	uint32_t read = 0;

	// Set single-ended analog input channel.
	setSEChannel(channel);
	delayus(3); // min delay: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC);    // SYNC command
	delayus(3);
	writeCMD(CMD_WAKEUP);  // WAKEUP command
	delayus(1); // min delay: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec

	// Set continuous mode.
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	waitDRDY();
	send8bit(CMD_RDATAC);

	// Start reading data
	for (int i = 0; i < numOfMeasure-3; ++i)
	{
		waitDRDY();
		buffer[0] = recieve8bit();
		buffer[1] = recieve8bit();
		buffer[2] = recieve8bit();
//		// construct 24 bit value
		read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
		read |= ((uint32_t)buffer[1] << 8);
		read |= buffer[2];
		if (read & 0x800000){
			read &= 0x00FFFFFF;
		}
		values[i] = read;
	}

	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

// Continuously acquire analog data from one differential analog input.
// Allows sampling of one differential input channel up to 30,000 SPS.
void scanDIFFChannelContinuous(uint8_t positiveCh, uint8_t negativeCh, uint32_t numOfMeasure, uint32_t *values)
{
	uint8_t buffer[3];
	uint32_t read = 0;
	uint8_t del = 8;

	// Set differential analog input channel.
	setDIFFChannel(positiveCh, negativeCh);
	delayus(del);

	// Set continuous mode.
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	waitDRDY();
	send8bit(CMD_RDATAC);
	delayus(del); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds

	// Start reading data.
	for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY();
		buffer[0] = recieve8bit();
		buffer[1] = recieve8bit();
		buffer[2] = recieve8bit();

		// construct 24 bit value
		read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
		read |= ((uint32_t)buffer[1] << 8);
		read |= buffer[2];
		if (read & 0x800000){
			read |= 0xFF000000;
		}
		values[i] = read;
		//printf("%f %i\n", (float)read/1670000, clock() - startTime); // TESTING
		delayus(del);
	}

	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
// User created delay_us
// Uses internal NUCLEO timer1 at 1 Mhz with max 0xffff
// 1 tick = 1 us
void delayus(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter
}

double m_PI = (3.14159265358979323846); //google told me to define pi like this in C
int max_samples = 1000; // this is arbitrary, can be whatever
int numOfMeasure = 200;

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
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

//    LL_SPI_Enable(SPI2);
  // Initialize Nucleo timer
  HAL_TIM_Base_Start(&htim3);
  setBuffer(true);
  setPGA(PGA_GAIN16); // Keep at PGA_GAIN8 for 400 mV Peak
  setDataRate(DRATE_30000);

  int server = 1;
  int prev_server = 1;

  double server1_thd_5min;
  double server2_thd_5min;
  int sample_count1;
  int sample_count2;

//  CalibrateICIarms5mV();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  {

	// Pulled from ads1256.h/.c author
	// Changed functionality to do continuous read, kept for loops
	////////////////////////////////////
	// Single Channel Continuous read //
	////////////////////////////////////
	uint32_t numOfMeasure = 400; // 80 seems goodish
	uint32_t samples[numOfMeasure];
    double cleaned_samples[numOfMeasure];
    double cycle_samples[numOfMeasure];

    uint8_t txbuf[64];

    // Start continous read and collect numOfMeasure samples
//	scanSEChannelContinuous(AIN0, numOfMeasure, samples);
    if(server == 1){
    	scanDIFFChannelContinuous(AIN0,AIN1,numOfMeasure,samples);
    	prev_server = 1;
    	server = 2;
    }else{
    	scanDIFFChannelContinuous(AIN2,AIN3,numOfMeasure,samples);
    	prev_server = 2;
    	server = 1;
    }

	writeCMD(CMD_SELFCAL);
	delayus(1000);

	// Start data processing then send to COM port
	for(int i = 0; i < numOfMeasure; i++){
		double value = ((double)(samples[i]))/16777220;
		if(value > 5){
			value = value-256;
		}//else if(value < 0.001){
//			continue;
//		}else{
		cleaned_samples[i] = value; 	// THIS WILL CHANGE DEPENDING ON OFFSET VOLTAGE
//			//For debugging individual readings
//		sprintf((char*)txbuf, "%f\r\n", value);
//		HAL_UART_Transmit(&huart3, txbuf, strlen((char*)txbuf), 10);
//		}
	}



	int N_raw = sizeof cleaned_samples / sizeof (cleaned_samples[0]);
	int start = 0; // for cycle detection
	int N = 0;     // number of samples in the detected cycle
	int cycles_per_sample = 72;
	double mPI = 3.14159265358979323846;
	double thd = 0;

	for (int i = 1; i < N_raw; i++)
	{
		if (cleaned_samples[i] >= 0 && cleaned_samples[i-1] < 0) // detect rising edge by checking for transition from negative to positive
		{
			start = i; //I want to solve for N (number of samples) as the element at the end minus the element at the start. this is the start
			N = cycles_per_sample; //i is the 'end' here. so I subtract the end from the start to get N

			memset(cycle_samples, 0, N);//making a new array for the DFT calculation. basically I want to copy the elements of one array into another array

			for (int j = 0; j < N; j++)
			{
				cycle_samples[j] = cleaned_samples[start + j]; // copy from raw sample values array to new array for dft calculation

			}

			double fundamental_amplitude = 0.0; // to store amplitude of the fundamental frequency
			int max_m = 25;
			for (int m = 1; m <= max_m; m++) {
				double real = 0.0;
				double imag = 0.0;
				// perform DFT for each harmonic up to m = 25
				for (int n = 0; n < N; n++) {
					double angle = 2 * mPI * m * n / N;
					real += cycle_samples[n] * cos(angle);
					imag -= cycle_samples[n] * sin(angle);
				}
				real *= 2.0 / N; // multiply the real part by the constant outside the summation
				imag *= 2.0 / N; // multiply the imaginary part by the constant outside the summation

				double amplitude = sqrt(real * real + imag * imag); // calculate amplitude

				if (m == 1)
				{
					fundamental_amplitude = amplitude; // store fundamental amplitude
				}

				else
				{
					if (m%2 == 1){
//						double harmonic_distortion = (amplitude / fundamental_amplitude) * 100.0; // calculate harmonic distortion percentage
			//            printf("Harmonic order %d: Distortion = %.2f%%\n", m, harmonic_distortion);
						thd += (amplitude)*(amplitude);

//						sprintf((char*)txbuf, "Harmonic order %d: Distortion = %.2f%%\r\n", m, harmonic_distortion);
//						HAL_UART_Transmit(&huart3, txbuf, strlen((char*)txbuf), 10);
//						HAL_Delay(10);
					}
				}
			}
			thd = sqrt(thd)/fundamental_amplitude*100;

			sprintf((char*)txbuf, "1x1x%dx%.0fp\r\n", prev_server, thd);
			HAL_UART_Transmit(&huart3, &txbuf, strlen((char*)txbuf), 10); //strlen((char*)txbuf)
//			sprintf((char*)txbuf, "  \r\n");
//			HAL_UART_Transmit(&huart3, txbuf, strlen((char*)txbuf), 10);
			HAL_Delay(10);
			break;
		}
	}
//
//	if(prev_server == 1){
//		server1_thd_5min += thd;
//		sample_count1++;
//		if(sample_count1 >= 1){
//			server1_thd_5min /= sample_count1;
//			sprintf((char*)txbuf, "1x1x%dx%.0fp\r\n", prev_server, server1_thd_5min);
//			HAL_UART_Transmit(&huart3, &txbuf, strlen((char*)txbuf), 10);
//	//		HAL_Delay(10);
//
//			sample_count1 = 0;
//			server1_thd_5min = 0;
//		}
//	}
//	if(prev_server == 2){
//		server2_thd_5min += thd;
//		sample_count2++;
//		if(sample_count2 >= 1){
//			server2_thd_5min /= sample_count2;
//			sprintf((char*)txbuf, "1x1x%dx%.0fp\r\n", prev_server, server2_thd_5min);
//			HAL_UART_Transmit(&huart3, &txbuf, strlen((char*)txbuf), 10);
//	//		HAL_Delay(10);
//
//			sample_count2 = 0;
//			server2_thd_5min = 0;
//		}
//	}



	HAL_Delay(5000); // Wait 5 second
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
