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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// global variables for SPI communication

// Please do NOT touch this function, it is working as intended
// Set CS to 0
// Transmit desired address from IC Chip (16-bits)
// Receive data from IC Chip (16-bits)
// Takes 16 bit address (LSByte first)
// Returns 16 bit data
uint16_t TransmitReceiveRead (uint16_t addr){
	uint16_t data;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&addr, 2, 100);
	HAL_SPI_Receive(&hspi1, (uint8_t *)&data, 2, 100);
	// Reformat data since bytes are read backwards
	uint8_t tempData1 = data >> 8 & 0xFF;
	uint8_t tempData2 = data & 0xFF;
	data = tempData2 << 8 | tempData1;
//	data = tempData1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	return data;
}

// Please do NOT touch this function, it is working as intended
// Set CS to 0
// Transmit desired address to IC Chip (16-bits)
// Transmit desired data to the IC Chip (16-bits)
// Takes 16 bit address (LSByte first)
// Takes 16 bit data (LSByte first)
void TransmitReceiveWrite (uint16_t addr, uint16_t data){
	// Reformat data since bytes are read backwards
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	uint8_t tempData1 = data >> 8 & 0xFF;
	uint8_t tempData2 = data & 0xFF;
	data = tempData2 << 8 | tempData1;

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&addr, 2, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 2, 100);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

// Calibration process listed in the M90E36A Datasheet
// WARNING: This is a WIP function, requirements of project
// specify no need to calibrate system
// Should return 6886H
uint16_t CalibrateICStartConfig (void){
	HAL_Delay(500);

	// See what's on checksum CS0 register
	uint16_t CS0_1;
	uint16_t CS0Addr = 0x3B80;
	CS0_1 = TransmitReceiveRead(CS0Addr);
	// Start configuring system config registers
	uint16_t configStart = 0x5678;
	uint16_t configStartAddr = 0x3000;
	TransmitReceiveWrite(configStartAddr, configStart);
	// Metering Method Config 1504, 1484
	uint16_t MMode0Addr = 0x3300;
	uint16_t MMode0 = 0x1504;
	TransmitReceiveWrite(MMode0Addr,MMode0);

	// PGA Gain Configuration, set all to 4 = AAAA, 2 = 5555, 1 = 0000
	uint16_t MMode1Addr = 0x3400;
	uint16_t MMode1 = 0xDAAA;
	TransmitReceiveWrite(MMode1Addr,MMode1);

	// Set DFT Configuration
	uint16_t DFT_Addr = 0xD001;
	//0x0049 is gain 2
	//0x0092 is gain 4
	//0x00DB is gain 8
	//0x0124 is gain 16
	//0x016D is gain 32
	//0x01B6 is gain 64
	//0x01FF is gain 128
	uint16_t DFT = 0x01FF;
	TransmitReceiveWrite(DFT_Addr,DFT);

	// Turn on checksum checking
	uint16_t startValue = 0x8765;
	uint16_t configStartRegAddr = 0x3000;
	TransmitReceiveWrite(configStartRegAddr, startValue);
	uint16_t calStartRegAddr = 0x4000;
	TransmitReceiveWrite(calStartRegAddr, startValue);
	uint16_t harmStartRegAddr = 0x5000;
	TransmitReceiveWrite(harmStartRegAddr, startValue);
	uint16_t adjStartRegAddr = 0x6000;
	TransmitReceiveWrite(adjStartRegAddr, startValue);


	HAL_Delay(500);
	// return checksum at start of power up state
	return CS0_1;
}

// Should return 8765H meaning operating properly
uint16_t CalibrateICIarms0mV (void){
	// Measurement calibration
	// *********************************
	// Calibrate Irms phase A
	unsigned int currentAMSB = 0;
	for (int i = 0; i < 10; i++){
		currentAMSB += TransmitReceiveRead(0xDD80);
		HAL_Delay(50);
	}
	unsigned int currentALSB = 0;
	for (int i = 0; i < 10; i++){
		currentALSB += TransmitReceiveRead(0xED80);
		HAL_Delay(50);
	}
	currentAMSB /= 10;
	currentALSB /= 10;
	uint32_t currentA = (currentAMSB << 16) | currentALSB;
	uint16_t currentAOffset = ((~(currentA >> 7))+0b1);
//	uint16_t currentAOffset = currentA | 0x8000;

	// Start configuring system measurement registers
	uint16_t configStart = 0x5678;
	uint16_t configStartAddr = 0x6000;
	TransmitReceiveWrite(configStartAddr, configStart);
	TransmitReceiveWrite(0x6400, currentAOffset);

	// Set gain calibration
	// Value based on 0.65A test
	// Then calculated as reference i/current measurement *30000
	TransmitReceiveWrite(0x6200, 0x0E91);

	uint8_t txbuf[64];
	sprintf((char*)txbuf, "iA offset: %d\r\n", currentAOffset);
	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
	// *********************************

	// Turn on all processing
	uint16_t startValue = 0x8765;
	uint16_t configStartRegAddr = 0x3000;
	TransmitReceiveWrite(configStartRegAddr, startValue);
	uint16_t calStartRegAddr = 0x4000;
	TransmitReceiveWrite(calStartRegAddr, startValue);
	uint16_t harmStartRegAddr = 0x5000;
	TransmitReceiveWrite(harmStartRegAddr, startValue);
	uint16_t adjStartRegAddr = 0x6000;
	TransmitReceiveWrite(adjStartRegAddr, startValue);

	// Give checksum time to calculate
	HAL_Delay(500);

	// See what's on checksum CS0 register
	uint16_t CS0Addr = 0x3B80;
	return TransmitReceiveRead(CS0Addr);
}

// Return sysStatus0 register
uint16_t sysStatus0(void){
	return (TransmitReceiveRead(0x0180));
}
// Return sysStatus1 register
uint16_t sysStatus1(void){
	return (TransmitReceiveRead(0x0280));
}

// function that transmits to required registers to:
// 	start measuring and calculating harmonic components
// From datasheet:
//a. Set DFT computation engine and write 2A49H to the DFT_SCALE [1D0H] register (Assume gain of voltage and current is two)
//b. Start DFT computation engine and write 001H to the DFT_CTRL [1D1H] register
//c. Check DFT_CTRL. If DFT_CTRL=0, DFT computation is completed (about 0.5s)
//d. Read register value and get harmonic component and fundamental voltage/current value after transition
void startDFT(void){
	// Start DFT computation engine (DFT_CTRL)
	TransmitReceiveWrite(0xD101,0x0001);
}

// return DFT check register
uint16_t checkDFT(void){
	return TransmitReceiveRead(0x0280);
}

// Return calculated frequency based on voltage input
float frequency(void){
	return (0.01*(int)TransmitReceiveRead(0xF880));
}

// Return calculated current on phase A
float currentA(void){
	float iA = 0;
	for(int i = 0; i<10; i++){
		iA += (int)TransmitReceiveRead(0xDD80);
	}
	iA = 0.001*(iA/10);
	float iALSB = 0;
	for(int i = 0; i<10; i++){
		iALSB += (int)TransmitReceiveRead(0xED80);
	}
	iALSB = (iALSB/10) * 0.001/256;
	return (iA + iALSB);
}

// Return calculated fundamental THD on phase A
float fundamentalIA(void){
	startDFT();
	uint16_t check = checkDFT();
	while(!((check)&&(0x0200))){
		check = checkDFT();
		HAL_Delay(100);
	}
	int funIA = (int)TransmitReceiveRead(0xF580);
	return (funIA * 3.2656)/(4*100);
}

// Return calculated THD+N (total THD) on phase A
float phaseIATHDN(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0xF580));
		startDFT();
	}
	return (0.01*(pIA/10));
}

// Return calculated THD ratio on phase A
float phaseIATHDRatio(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0x1F81));
		startDFT();
	}
	pIA /= 10;
//	uint16_t check = checkDFT();
//	while(!((check)&&(0x0200))){
//		check = checkDFT();
//		HAL_Delay(100);
//	}
	return ((int)pIA/163.84);
}

// 1-data ************************************************************
// Return calculated 3rd THD on phase A
//float phaseIA3rdTHD(void){
//	startDFT();
//	float pIA = 0;
////	for(int i = 0; i<10; i++){
////		uint16_t check = checkDFT();
////		while(!((check)&&(0x0200))){
////			check = checkDFT();
////			HAL_Delay(100);
////		}
////		pIA += ((int)TransmitReceiveRead(0x0181));
////		startDFT();
////	}
////	pIA /= 10;
//	uint16_t check = checkDFT();
//	while(!((check)&&(0x0200))){
//		check = checkDFT();
//		HAL_Delay(100);
//	}
//	pIA += ((int)TransmitReceiveRead(0x0181));
//	return ((int)pIA/163.84);
//}
//
//// Return calculated 5th THD on phase A
//float phaseIA5thTHD(void){
//	startDFT();
//	float pIA = 0;
////	for(int i = 0; i<10; i++){
////		uint16_t check = checkDFT();
////		while(!((check)&&(0x0200))){
////			check = checkDFT();
////			HAL_Delay(100);
////		}
////		pIA += ((int)TransmitReceiveRead(0x0381));
////		startDFT();
////	}
////	pIA /= 10;
//	uint16_t check = checkDFT();
//	while(!((check)&&(0x0200))){
//		check = checkDFT();
//		HAL_Delay(100);
//	}
//	pIA += ((int)TransmitReceiveRead(0x0381));
//	return ((int)pIA/163.84);
//}
//
//// Return calculated 7th THD (total THD) on phase A
//float phaseIA7thTHD(void){
//	startDFT();
//	float pIA = 0;
////	for(int i = 0; i<10; i++){
////		uint16_t check = checkDFT();
////		while(!((check)&&(0x0200))){
////			check = checkDFT();
////			HAL_Delay(100);
////		}
////		pIA += ((int)TransmitReceiveRead(0x0581));
////		startDFT();
////	}
////	pIA /= 10;
//	uint16_t check = checkDFT();
//	while(!((check)&&(0x0200))){
//		check = checkDFT();
//		HAL_Delay(100);
//	}
//	pIA += ((int)TransmitReceiveRead(0x0581));
//	return ((int)pIA/163.84);
//}
// 1-data ************************************************************

//// 10-data AVERAGE ************************************************************
// Return calculated 3rd THD on phase A
float phaseIA3rdTHD(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0x0181));
		startDFT();
	}
	pIA /= 10;
//	uint16_t check = checkDFT();
//	while(!((check)&&(0x0200))){
//		check = checkDFT();
//		HAL_Delay(100);
//	}
	return ((int)pIA/163.84);
}

// Return calculated 5th THD on phase A
float phaseIA5thTHD(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0x0381));
		startDFT();
	}
	pIA /= 10;
//	uint16_t check = checkDFT();
//	while(!((check)&&(0x0200))){
//		check = checkDFT();
//		HAL_Delay(100);
//	}
	return ((int)pIA/163.84);
}

// Return calculated 7th THD (total THD) on phase A
float phaseIA7thTHD(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0x0581));
		startDFT();
	}
	pIA /= 10;
//	uint16_t check = checkDFT();
//	while(!((check)&&(0x0200))){
//		check = checkDFT();
//		HAL_Delay(100);
//	}
	return ((int)pIA/163.84);
}

// Return calculated 9th THD (total THD) on phase A
float phaseIA9thTHD(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0x0781));
		startDFT();
	}
	pIA /= 10;
	return ((int)pIA/163.84);
}

// Return calculated 11th THD (total THD) on phase A
float phaseIA11thTHD(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0x0981));
		startDFT();
	}
	pIA /= 10;
	return ((int)pIA/163.84);
}

// Return calculated 13th THD (total THD) on phase A
float phaseIA13thTHD(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0x0B81));
		startDFT();
	}
	pIA /= 10;
	return ((int)pIA/163.84);
}

// Return calculated 15th THD (total THD) on phase A
float phaseIA15thTHD(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0x0D81));
		startDFT();
	}
	pIA /= 10;
	return ((int)pIA/163.84);
}
//// 10-data AVERAGE ************************************************************

// Current B calculation registers
// Return calculated THD+N (total THD) on phase A
float phaseIBTHDN(void){
	startDFT();
	float pIA = 0;
	for(int i = 0; i<10; i++){
		uint16_t check = checkDFT();
		while(!((check)&&(0x0200))){
			check = checkDFT();
			HAL_Delay(100);
		}
		pIA += ((int)TransmitReceiveRead(0xF680));
		startDFT();
	}
	return (0.01*(pIA/10));
}

// Return calculated THD ratio on phase A
float phaseIBTHDRatio(void){
	startDFT();
	uint16_t check = checkDFT();
	while(!((check)&&(0x0200))){
		check = checkDFT();
		HAL_Delay(100);
	}
	return ((int)TransmitReceiveRead(0x3F81)/163.84);
}

// Return calculated 3rd THD on phase A
float phaseIB3rdTHD(void){
	startDFT();
	uint16_t check = checkDFT();
	while(!((check)&&(0x0200))){
		check = checkDFT();
		HAL_Delay(100);
	}
	return ((int)TransmitReceiveRead(0x2181)/163.84);
}

// Return calculated 5th THD on phase A
float phaseIB5thTHD(void){
	startDFT();
	uint16_t check = checkDFT();
	while(!((check)&&(0x0200))){
		check = checkDFT();
		HAL_Delay(100);
	}
	return ((int)TransmitReceiveRead(0x2381)/163.84);
}

// Return calculated 7th THD (total THD) on phase A
float phaseIB7thTHD(void){
	startDFT();
	uint16_t check = checkDFT();
	while(!((check)&&(0x0200))){
		check = checkDFT();
		HAL_Delay(100);
	}
	return ((int)TransmitReceiveRead(0x2581)/163.84);
}

// Return calculated current on phase A
float PMcurrentA(void){
	uint16_t data = 0x4001;
	float iA = 0;
	for(int i = 0; i<10; i++){
		TransmitReceiveWrite(0x1B00,data);
		iA += (int)TransmitReceiveRead(0x1880);
		HAL_Delay(500);
	}
	iA = (iA/10);
	return (iA);
}

// Return calculated current on phase B
float currentB(void){
	return (0.001*(int)TransmitReceiveRead(0xDE80));
}

// Return calculated current on phase C
float currentC(void){
	return (0.001*(int)TransmitReceiveRead(0xDF80));
}

// calibrate IC onchip temperature sensor
void calTemperature(void){
	TransmitReceiveWrite(0xFD02, 0xAA55);
	TransmitReceiveWrite(0x1602, 0x5122);
	TransmitReceiveWrite(0x1902, 0x012B);
	TransmitReceiveWrite(0xFD02, 0x0000);
}

// returns temperature C
uint16_t temperatureC(void){
	return TransmitReceiveRead(0xFC80);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	VIRUTAL COM PORT TEST
//	uint8_t Test[] = "Hello World !!!\r\n"; //Data to send
//	HAL_UART_Transmit(&huart2,Test,sizeof(Test),10);// Sending in normal mode
//	HAL_Delay(1000);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint16_t sys0;
  uint16_t sys1;
  uint16_t test;
  int16_t temp;
  float THD_A_3rd;
  float THD_A_5th;
  float THD_A_7th;
  float THD_A_9th;
  float THD_A_11th;
  float THD_A_13th;
  float THD_A_15th;
  float calcCurrA;
  float calcCurrB;
  float calcCurrC;
  float aTHDN;
  float THD_A;
  float THD_B_3rd;
  float THD_B_5th;
  float THD_B_7th;
  float bTHDN;
  float THD_B;
  float freq;

  uint16_t check;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //PM0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //PM1

  // Software Reset
  TransmitReceiveWrite(0x0080,0x789A);
  // Set IC startconfig to measure 60 Hz
  CalibrateICStartConfig();
  calTemperature();
  CalibrateICIarms0mV();

//  CalibrateICIarms5mV();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  {
	// test SPI is setup and reading correctly
	// 8 means read address, 0 means write to address
	test = TransmitReceiveRead(0x0E80); // Should return 7E44H

	startDFT();
	check = checkDFT();
	while(!((check)&&(0x0200))){
		check = checkDFT();
		HAL_Delay(500);
	}

	sys0 = sysStatus0();
	sys1 = sysStatus1();

//	aTHDN = phaseIATHDN();
//	THD_A = phaseIATHDRatio();
//	calcCurrA = currentA();
//	THD_A_3rd = phaseIA3rdTHD();
//	THD_A_5th = phaseIA5thTHD();
//	THD_A_7th = phaseIA7thTHD();
//	THD_A_9th = phaseIA9thTHD();
//	THD_A_11th = phaseIA11thTHD();
//	THD_A_13th = phaseIA13thTHD();
//	THD_A_15th = phaseIA15thTHD();
//
//	uint8_t txbuf[64];
//	sprintf((char*)txbuf, "currentA: %.2f\r\n", calcCurrA);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "THD_A: %.2f\r\n", THD_A);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "THD_3RD_A: %.2f\r\n", THD_A_3rd);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "THD_5TH_A: %.2f\r\n", THD_A_5th);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "THD_7TH_A: %.2f\r\n", THD_A_7th);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "THD_9TH_A: %.2f\r\n", THD_A_9th);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "THD_11TH_A: %.2f\r\n", THD_A_11th);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "THD_13TH_A: %.2f\r\n", THD_A_13th);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "THD_15TH_A: %.2f\r\n", THD_A_15th);
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	sprintf((char*)txbuf, "\r\n");
//	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
//	HAL_Delay(200); // delay 0.2 sec

//	bTHDN = phaseIBTHDN();
//	THD_B = phaseIBTHDRatio();
//	THD_B_3rd = phaseIB3rdTHD();
//	THD_B_5th = phaseIB5thTHD();
//	calcCurrB = currentB();
//	temp = temperatureC();

	// ********* USB Serial TEST DATA *********
	bTHDN = 10.5;
	THD_B = 175.5;
	THD_B_3rd = 90.1;
	THD_B_5th = 73.3;
	calcCurrB = 54.5;
	temp = 17;
	// ********* USB Serial TEST DATA *********

	uint8_t txbuf[64];
	sprintf((char*)txbuf, "Temperature: %.2f\r\n", temp);
	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
	sprintf((char*)txbuf, "currentB: %.2f\r\n", calcCurrB);
	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
	sprintf((char*)txbuf, "THD_B: %.2f\r\n", THD_B);
	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
	sprintf((char*)txbuf, "THD_3RD_B: %.2f\r\n", THD_B_3rd);
	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
	sprintf((char*)txbuf, "THD_5TH_B: %.2f\r\n", THD_B_5th);
	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
	sprintf((char*)txbuf, "\r\n");
	HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), 10);
	HAL_Delay(1000); // delay test data by 1 second
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
