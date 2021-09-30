/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
//#include "MCP6S2.h"
#include "INA226.h"
#include "stream_buffer.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_class.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId_t defaultTaskHandle;
osThreadId_t processUSBTaskHandle;

USBD_HandleTypeDef hUsbDeviceFS;
StreamBufferHandle_t xStreamBuffer = NULL;

// GLOBAL ERROR
uint8_t gError = 0;

uint8_t running = 0;

// cyclesElapsed
uint32_t cyclesElapsed = 0;
uint32_t prevCycle = 0;

#define TIMER_MIN	0

// active test flags; all off
uint8_t bounceTest = 0;
uint8_t currentTest = 0;
uint8_t resistTest = 0;

int bounceTimer = 5;	// countdown number of cycles unitl next bounce test
int currentTimer = 4;	// countdown number of cycles unitl next coil current test
int resistTimer = 3;	// countdown number of cycles unitl next coil ressitance test
// -----------------------------------------------

// TEST RESULTS
// -----------------------------------------------
// INA226
uint8_t i2cBuf[3];
uint32_t i2cresults[4];	// results array: Bus Voltage, Shunt Voltage, Bus Current, Bus Power

// bounce data
uint16_t adc_buf[ADC_BUF_LEN];	// DMA buffer
uint8_t bounceDataReady = 0;	// bounce data ready flag

uint16_t coilResist = 0;

volatile uint8_t triggered = 0;		// activation/deactivation time interrupt flag
volatile uint16_t switchTime = 0;	// activation/deactivation time (timer ticks)

// -----------------------------------------------

// USB update packets
int usbPacketLen = 0;	// length of packet

// time
//char time[13];
//char date[6];

// timestamp struct
struct timestamp {
	uint16_t hours;
	uint8_t mins;
	uint8_t secs;
	uint8_t msecs;
} diffTime;

//struct timestamp startTime, endTime, currTime, prevTime;

// Update struct:
//
// timestamp:
//    timestamp (4 bytes),
// cycle number (4 bytes)
// activation time in us (4 bytes),
// deactivation time in us (4 bytes),
// INA226 bus voltage (2 bytes),
// INA226 shunt voltage (2 bytes),
// INA226 bus current (2 bytes),
// INA226 bus power (2 bytes),
// 12-bit coil resistance (2 bytes, padded left)
// Total: 40 bytes
struct updatePacket {
	struct timestamp ts;		// timestamp
	uint32_t cycle;				// current cycle number
	uint32_t act_t;				// activation time
	uint32_t deact_t;			// deactivation time
	uint16_t busV;				// INA226 bus voltage
	uint16_t shuntV;			// INA226 shunt voltage
	uint16_t busI;				// INA226 bus current
	uint16_t busP;				// INA226 bus power
	uint16_t coilR;				// coil resistance
	uint8_t bounceReady;		// bounce data ready flag
} infoUpdatePacket;

// USB update packet struct
// includes header and footer, as well as the new flag
struct USBUpdatePacket {
	uint8_t packetHeader;			// packet type; typically FLAG_UPDATE_START
	uint8_t newData;				// indicated if the data in the packet is new (different timestamp)
	struct updatePacket data;		// update packet struct containing update data
	uint8_t packetFooter;			// packet footer; typically FLAG_UPDATE_END
} currUSBUpdatePacket;

// Config struct:
//
// relay coil voltage setting (4 bytes),
// test frequency (4 bytes),
// number of cycles (4 bytes),
// length of test (timestamp):
//	   timestamp (4 bytes)
// bounce test period (in cycles) (2 bytes),
// coil current test period (in cycles) (2 bytes),
// coil resistance test period (in cycles) (2 bytes),
// number of failures before stop (1 byte)
// Total: 36 bytes
struct configPacket {
	float coilV;				// coil voltage (0.0 means test for it)
	float testPeriod;				// test period (ms)
	uint32_t numCycles;			// number of cycles in test
	struct timestamp end_t;		// end time timestamp
	uint16_t bounceTestPer;		// bounce test period
	uint16_t coilCurrentTestPer;// coil current test period
	uint16_t coilResistTestPer; // coil resistance test period
	uint16_t numFail;			// number of failures before test is auto stopped
} testParams;	// create struct to hold test parameters

// USB config packet struct
// includes header and footer
struct USBConfigPacket {
	uint8_t packetHeader;			// packet type; typically FLAG_CONFIG_START
	struct configPacket data;		// update packet struct containing config data
	uint8_t packetFooter;			// packet footer; typically FLAG_CONFIG_END
} newTestParamsPacket;

// USB buffer for lower-level xStreamBuffer
uint8_t usbBuffer[512];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void *argument);
void StartProcessUSB(void *argument);

/* USER CODE BEGIN PFP */
void DMATransferComplete(DMA_HandleTypeDef *hdma);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler(void) {
	/* USER CODE BEGIN EXTI4_IRQn 0 */

	HAL_TIM_Base_Stop(&htim6);			// stop timer
	switchTime = htim6.Instance->CNT;	// record counter value
	triggered = 1;

	/* USER CODE END EXTI4_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
	/* USER CODE BEGIN EXTI4_IRQn 1 */

	/* USER CODE END EXTI4_IRQn 1 */
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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_DAC_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_USART3_UART_Init();
	MX_RTC_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_TIM6_Init();

	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(PS_EN_GPIO_Port, PS_EN_Pin, GPIO_PIN_RESET);	// disable power supply

	USBD_Init(&hUsbDeviceFS, &Class_Desc, 0);
	USBD_RegisterClass(&hUsbDeviceFS, &USBD_TEMPLATE_ClassDriver);
	USBD_Start(&hUsbDeviceFS);

	// I2C to INA226
	// set configuration
	uint16_t INAconfig = (0x4000 | INA226_AVG_64 | INA226_VBUS_2116uS
			| INA226_VSH_2116uS | INA226_MODE_CONT);
	i2cBuf[0] = 0x00;
	i2cBuf[1] = INAconfig >> 8;
	i2cBuf[2] = INAconfig;
	HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_WRITE, i2cBuf, 3,
	INA226_I2C_TO);

	// set calibration
	i2cBuf[0] = 0x05;
	i2cBuf[1] = 0x11;
	i2cBuf[2] = 0x7A;
	HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_WRITE, i2cBuf, 3,
	INA226_I2C_TO);

	uint16_t spiBuf[1];
	spiBuf[0] = 0b0100000000000101;
	HAL_SPI_Transmit(&hspi2, &spiBuf, 1, 50);	// set gain to 8

	//HAL_SPI_Transmit(&hspi2, uint8_t * pData, uint16_t Size, uint32_t Timeout)

//	HAL_SPI_Transmit(&hspi2, (uint8_t*)&buf, 1, 100);

	// MCP6S2x
	//configure bounce gain to 10x
//	setGain(10);

	// DAC init
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4096);    // 0V

	// RTC
//	setTime();

	// default
	testParams.testPeriod = 1000;

	/*****For USB****
	 * HAL_DMA_RegisterCallback(&hdma_usart3_tx, HAL_DMA_XFER_CPLT_CB_ID, &DMATransferComplete);
	 */
	//Start ADC with DMA transfer
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	xStreamBuffer = xStreamBufferCreate(512, 1);
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	const osThreadAttr_t defaultTask_attributes = {
			.name = "defaultTask",
			.priority = (osPriority_t) osPriorityNormal,
			.stack_size = 256
	};
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* definition and creation of invertAsciiTask */
	const osThreadAttr_t processUSB_attributes = {
			.name =	"processUSBTask",
			.priority = (osPriority_t) osPriorityHigh,
			.stack_size = 768
	};
	processUSBTaskHandle = osThreadNew(StartProcessUSB, NULL,
			&processUSB_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		// DO NOT PUT ANYTHING IN HERE
		// ALL LOOPED CODE SHOULD GO IN DEFAULT TASK FUNCTION

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// NO!!! (╯°□°)╯︵ ┻━┻
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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC
			| RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void) {

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */
	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT2 config
	 */
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

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
	hi2c1.Init.Timing = 0x20404768;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 107;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, TRIG_OUT_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PS_EN_GPIO_Port, PS_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TRIG_OUT_Pin */
	GPIO_InitStruct.Pin = TRIG_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TRIG_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TRIG_IN_Pin */
	GPIO_InitStruct.Pin = TRIG_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TRIG_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PS_EN_Pin */
	GPIO_InitStruct.Pin = PS_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(PS_EN_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//For USB
void DMATransferComplete(DMA_HandleTypeDef *hdma) {
	//Disable UART DMA mode
//	huart3.Instance->CR3 &= ~USART_CR3_DMAT;
}

/*//Called when first half of buffer is filled, turns on LED
 void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
 }

 //Called when buffer it totally full, turns off LED
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
 }*/

//function for data packets for USART
//packets will have a 2 bit start/end opcode, 2 bit opcode for what type of data (bounce info, coil resistance, etc) and the data
//need defines for data types
//void USART_Packet(uint8_t dType, uint32_t msg) {
//
//}


// compare two timestamps
// returns:
//		-1: time1 is OLDER than time2
//		0: time1 is the same as time2
//		1: time1 is NEWER than time2
//int compareTime(struct timestamp *time1, struct timestamp *time2) {
//
//	if (time1->hours > time2->hours) {
//		return 1;
//	} else if (time1->hours < time2->hours) {
//		return -1;
//	} else {
//		if (time1->mins > time2->mins) {
//			return 1;
//		} else if (time1->mins < time2->mins) {
//			return -1;
//		} else {
//			if (time1->secs > time2->secs) {
//				return 1;
//			} else if (time1->secs < time2->secs) {
//				return -1;
//			} else {
//				if (time1->msecs > time2->msecs) {
//					return 1;
//				} else if (time1->msecs < time2->msecs) {
//					return -1;
//				} else {
//					return 0;
//				}
//			}
//		}
//	}
//}

//Set RTC time
//void setTime(void) {
//	RTC_TimeTypeDef sTime;
//	RTC_DateTypeDef sDate;
//	/*set time*/
//	sTime.Hours = 0x00;			//set hours
//	sTime.Minutes = 0x00;		//set minutes
//	sTime.Seconds = 0x00;		//set seconds
//	sTime.SubSeconds = 0x00;	//set sub seconds
//	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//		Error_Handler();
//
//	/*set date*/
//	sDate.Date = 0x00;					//date
//	sDate.Month = RTC_MONTH_JANUARY;//month (how to set this to current month??)
//	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//		Error_Handler();
//
//	//assign backup register to maintain time after system reset
////	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32f2);
//}

//get RTC time
//void getTime(struct timestamp *times) {
//	RTC_TimeTypeDef gTime;
//	RTC_DateTypeDef gDate;
//	//get current time
//	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
//	//get current date
//	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
//	//display time
////	sprintf((char*) time, "%02d:%02d:%02d:%02d", gTime.Hours, gTime.Minutes,
////			gTime.Seconds, gTime.SubSeconds);
////	//display date
////	sprintf((char*) date, "%02d %02d", gDate.Month, gDate.Date);
//
//
//	// put into struct
//	times->hours = gTime.Hours + (gDate.Date * 24);
//	times->mins = gTime.Minutes;
//	times->secs = gTime.Seconds;
//	times->msecs = gTime.SubSeconds;
//}
//
//struct timestamp actTimeStart;

// computer time difference between two timestamps
//void computeTimeDiff(struct timestamp *diffTime, struct timestamp *time1, struct timestamp *time2) {
//	diffTime->hours = time1->hours - time2->hours;
//	diffTime->mins = time1->mins - time2->mins;
//	diffTime->secs = time1->secs - time2->secs;
//	diffTime->msecs = time1->msecs - time2->msecs;
//}


// pack data into packet data format
void packUpdate() {

	//get RTC time
//	prevTime = currTime;
//	getTime(&currTime);
//	computeTimeDiff(&diffTime, &currTime, &startTime);

//	char timeStr[24];
//	sprintf(timeStr, "%d:%d:%d:%d\n", diffTime.hours, diffTime.mins, diffTime.secs, diffTime.msecs);
//	HAL_UART_Transmit(&huart3, timeStr, 24, 100);

	currUSBUpdatePacket.packetHeader = FLAG_UPDATE_START; // update start flag

	currUSBUpdatePacket.packetHeader = FLAG_UPDATE_START; // update start flag

	// INA226 data

	// bus voltage
	infoUpdatePacket.busV = i2cresults[0];

	// shunt voltage
	infoUpdatePacket.shuntV = i2cresults[1];

	// bus current
	infoUpdatePacket.busI = i2cresults[2];

	// bus power
	infoUpdatePacket.busP = i2cresults[3];

	// current cycle
	infoUpdatePacket.cycle = cyclesElapsed;

	// bounce data ready flag
	infoUpdatePacket.bounceReady = bounceDataReady;
	bounceDataReady = 0;

	currUSBUpdatePacket.data = infoUpdatePacket;
//	infoUpdatePacket.ts = diffTime;
//	infoUpdatePacket.ts.hours = (uint16_t)((uint32_t)(cyclesElapsed+1) * (uint32_t)testParams.testPeriod) / 3600000;
//	infoUpdatePacket.ts.mins = (uint16_t)((uint32_t)(cyclesElapsed+1) * (uint32_t)testParams.testPeriod) / 60000;
//	infoUpdatePacket.ts.secs = (uint16_t)((uint32_t)(cyclesElapsed+1) * (uint32_t)testParams.testPeriod) / 1000;
//	infoUpdatePacket.ts.msecs = 1;

	currUSBUpdatePacket.packetFooter = FLAG_UPDATE_END; // update end flag

	// update new data flag
//	if (compareTime(&currTime, &prevTime) > 0) {
//		currUSBUpdatePacket.newData = FLAG_UPDATE_NEW;
//	} else {
//		currUSBUpdatePacket.newData = FLAG_UPDATE_OLD;
//	}
	if (cyclesElapsed != prevCycle) {
		currUSBUpdatePacket.newData = FLAG_UPDATE_NEW;
	} else {
		currUSBUpdatePacket.newData = FLAG_UPDATE_OLD;
	}
}

uint8_t nextState() {

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {

		if (running) {
			osDelay((newTestParamsPacket.data.testPeriod*0.25) - 50);

			if (bounceTimer == 0 && bounceTest == 1) {
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);				// start ADC1
				osDelay(2);
				for (uint16_t i = 0; i < 2500; i++) {

				}
			}

			// activation time test
			htim6.Instance->CNT = TIMER_MIN;										// reset time value to min
			HAL_GPIO_WritePin(TRIG_OUT_GPIO_Port, TRIG_OUT_Pin, GPIO_PIN_SET);		// trigger pin HIGH
			HAL_TIM_Base_Start(&htim6);

			while (triggered == 0) {		// wait for trigger
				osDelay(1);
			}

			infoUpdatePacket.act_t = (uint32_t)switchTime;							// record switch time

			if (bounceTimer == 0 && bounceTest == 1) {
				osDelay(3);
				HAL_ADC_Stop_DMA(&hadc1);				// stop ADC1
				bounceDataReady = 2;					// set flag
				bounceTimer = testParams.bounceTestPer;	// reset bounce timer
			} else {
				bounceTimer--;
			}

			triggered = 0;		// reset

			osDelay(newTestParamsPacket.data.testPeriod*0.75);

			// get Bus Voltage
			i2cBuf[0] = 0x02;
			HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_READ, i2cBuf, 3,
			INA226_I2C_TO);
			HAL_I2C_Master_Receive(&hi2c1, INA226_ADDRESS_READ, &i2cBuf[1], 2,
			INA226_I2C_TO);

			i2cresults[0] = (uint16_t) i2cBuf[1] << 8 | i2cBuf[2];

			// get shunt Voltage
			i2cBuf[0] = 0x01;
			HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_READ, i2cBuf, 3,
			INA226_I2C_TO);
			HAL_I2C_Master_Receive(&hi2c1, INA226_ADDRESS_READ, &i2cBuf[1], 2,
			INA226_I2C_TO);

			i2cresults[1] = (uint16_t) i2cBuf[1] << 8 | i2cBuf[2];

			// get Bus Power
			i2cBuf[0] = 0x03;
			HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_READ, i2cBuf, 3,
			INA226_I2C_TO);
			HAL_I2C_Master_Receive(&hi2c1, INA226_ADDRESS_READ, &i2cBuf[1], 2,
			INA226_I2C_TO);

			i2cresults[3] = (uint16_t) i2cBuf[1] << 8 | i2cBuf[2];

			// get Bus Current
			i2cBuf[0] = 0x04;
			HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_READ, i2cBuf, 3,
			INA226_I2C_TO);
			HAL_I2C_Master_Receive(&hi2c1, INA226_ADDRESS_READ, &i2cBuf[1], 2,
			INA226_I2C_TO);

			i2cresults[2] = (uint16_t) i2cBuf[1] << 8 | i2cBuf[2];

			// deactivation time test
			htim6.Instance->CNT = TIMER_MIN;										// reset time value to min
			HAL_GPIO_WritePin(TRIG_OUT_GPIO_Port, TRIG_OUT_Pin, GPIO_PIN_RESET);	// trigger pin LOW
			HAL_TIM_Base_Start(&htim6);

			while (triggered == 0) {		// wait for trigger
			}

			infoUpdatePacket.deact_t = (uint32_t)switchTime;
			triggered = 0;		// reset

			if (currentTimer == 0 && currentTest == 1) {
//				HAL_UART_Transmit(&huart3, "CURRENT TEST!\n", 13, USART_TIMEOUT);
				currentTimer = testParams.coilCurrentTestPer;
			} else {
				currentTimer--;
			}

			if (resistTimer == 0 && resistTest == 1) {
//				HAL_UART_Transmit(&huart3, "RESIST TEST!\n", 13, USART_TIMEOUT);
				resistTimer = testParams.coilResistTestPer;
			} else {
				resistTimer--;
			}

			prevCycle = cyclesElapsed;		// save previous cycle number
			cyclesElapsed++;				// increment cycles count

			if (cyclesElapsed < 10) {
//				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3100);	// 5V
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 835);    // 12V
			} else if (cyclesElapsed >= 10 && cyclesElapsed < 20) {
//				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3120);	// 5V
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 855);    // 12V
			} else if (cyclesElapsed >= 20 && cyclesElapsed < 30) {
//				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3140);	// 5V
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 875);    // 12V
			} else if (cyclesElapsed >= 30 && cyclesElapsed < 40) {
//				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3150);	// 5V
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 900);    // 12V
			}

		} else {
			osDelay(500);

			// get Bus Voltage
			i2cBuf[0] = 0x02;
			HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_READ, i2cBuf, 3,
			INA226_I2C_TO);
			HAL_I2C_Master_Receive(&hi2c1, INA226_ADDRESS_READ, &i2cBuf[1], 2,
			INA226_I2C_TO);

			i2cresults[0] = (uint16_t) i2cBuf[1] << 8 | i2cBuf[2];

			// get shunt Voltage
			i2cBuf[0] = 0x01;
			HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_READ, i2cBuf, 3,
			INA226_I2C_TO);
			HAL_I2C_Master_Receive(&hi2c1, INA226_ADDRESS_READ, &i2cBuf[1], 2,
			INA226_I2C_TO);

			i2cresults[1] = (uint16_t) i2cBuf[1] << 8 | i2cBuf[2];

			// get Bus Power
			i2cBuf[0] = 0x03;
			HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_READ, i2cBuf, 3,
			INA226_I2C_TO);
			HAL_I2C_Master_Receive(&hi2c1, INA226_ADDRESS_READ, &i2cBuf[1], 2,
			INA226_I2C_TO);

			i2cresults[3] = (uint16_t) i2cBuf[1] << 8 | i2cBuf[2];

			// get Bus Current
			i2cBuf[0] = 0x04;
			HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDRESS_READ, i2cBuf, 3,
			INA226_I2C_TO);
			HAL_I2C_Master_Receive(&hi2c1, INA226_ADDRESS_READ, &i2cBuf[1], 2,
			INA226_I2C_TO);

			i2cresults[2] = (uint16_t) i2cBuf[1] << 8 | i2cBuf[2];
		}

		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);	// flash green LED

		// Update the USB packet with latest data
		packUpdate();
	}
}

		//reading from ADC1 with DMA (for USart/USB)
		//huart3.Instance->CR3 |= USART_CR3_DMAT;

		/****For USB*****
		 HAL_DMA_Start_IT(&hdma_usart3_tx, msg,(uint32_t)&huart3.Instance->TDR, strlen(msg));
		 */

		//get ADC2 Value
		//HAL_ADC_Start(&hadc2);
		//HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		//reading = HAL_ADC_GetValue(&hadc2);


/* USER CODE BEGIN Header_StartProcessUSB */
/**
 * @brief Function implementing the processUSB thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartProcessUSB */
void StartProcessUSB(void *argument) {
	/* USER CODE BEGIN StartInvertAsciiTask */
	uint8_t sendResponse = 0;

	/* Infinite loop */
	for (;;) {
		size_t count = xStreamBufferReceive(xStreamBuffer, usbBuffer, 512, 2);
		if (!count)
			continue;

		// parse incoming command from PC
		if (count == 4) {
			switch (usbBuffer[0]) {

			case CMD_START:
				//HAL_UART_Transmit(&huart3, "Start command\n", 14, USART_TIMEOUT);

				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);	// flash red LED
//				getTime(&startTime);
//				diffTime.hours = 0;
//				diffTime.mins = 0;
//				diffTime.secs = 0;
//				diffTime.msecs = 0;

				cyclesElapsed = 0;

				HAL_GPIO_WritePin(PS_EN_GPIO_Port, PS_EN_Pin, GPIO_PIN_SET);	// enable power supply
//				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3100);    // 5.5V
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 860);    // 12.25V

//				osDelay(20);

				running = 1;
				break;

			case CMD_STOP:
				//HAL_UART_Transmit(&huart3, "Stop command\n", 13, USART_TIMEOUT);

				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);	// flash red LED

				running = 0;
				break;

			case CMD_GET_UPDATE:
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);	// flash Blue LED
				//HAL_UART_Transmit(&huart3, "Get update command\n", 18, USART_TIMEOUT);

				count = USB_UPDATE_LEN;

				memcpy(&usbBuffer, &currUSBUpdatePacket, USB_UPDATE_LEN); // copy new packet data to buffer

				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);	// flash Blue LED

				sendResponse = 1;

				break;

			case CMD_GET_BOUNCE_DATA:
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);	// flash Blue LED
//				HAL_UART_Transmit(&huart3, "Get bounce command\n", 19, USART_TIMEOUT);

				uint8_t bounceIndex = usbBuffer[1];

				count = 500;

//				memcpy(&usbBuffer, (uint8_t)&adc_buf, 500);

				// get data from ADC buffer
				for (uint16_t k = 0; k < 500; k++) {
					usbBuffer[k] = (uint8_t)adc_buf[k+3500+((bounceIndex-1)*500)];		// + 0 for 5V, +3000 for 12V
				}

				uint8_t status = USBD_TEMPLATE_Transmit(&hUsbDeviceFS, usbBuffer, count);
				while (status != USBD_OK){

				}
				osDelay(2);

				memcpy(&usbBuffer, &currUSBUpdatePacket, 36); // copy new packet data to buffer

				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);	// flash Blue LED

				sendResponse = 0;

				break;

			case CMD_CHECK_CONN:
				usbBuffer[3] = STM32_OK;
				//HAL_UART_Transmit(&huart3, "Connection check\n", 17, USART_TIMEOUT);
				sendResponse = 1;
				break;

			default:
				usbBuffer[3] = STM32_UNKNOWN;
				//HAL_UART_Transmit(&huart3, "Unknown command!\n", 17, USART_TIMEOUT);
				sendResponse = 1;
				break;
			}
		} else if (count == USB_CONFIG_LEN) {
			if (usbBuffer[0] == CMD_SET_CONFIG) {

				if (usbBuffer[1] == FLAG_CONFIG_START && usbBuffer[USB_CONFIG_LEN - 1] == FLAG_CONFIG_END) {
					//HAL_UART_Transmit(&huart3, "Config received\n", 16, USART_TIMEOUT);

					HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);	// flash red LED twice
					osDelay(400);
					HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);	// flash red LED twice
					osDelay(200);
					HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);	// flash red LED twice
					osDelay(400);
					HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);	// flash red LED twice

					memcpy(&newTestParamsPacket, usbBuffer+1, 33);

//					if (testParams.testPeriod != newTestParamsPacket.data.testPeriod) {
//						HAL_UART_Transmit(&huart3, "New test period\n", 16, USART_TIMEOUT);
//					}

					testParams.testPeriod = newTestParamsPacket.data.testPeriod;
					testParams.numCycles = newTestParamsPacket.data.numCycles;
					testParams.end_t = newTestParamsPacket.data.end_t;
					testParams.coilV = newTestParamsPacket.data.coilV;
					testParams.bounceTestPer = newTestParamsPacket.data.bounceTestPer;
					testParams.coilCurrentTestPer = newTestParamsPacket.data.coilCurrentTestPer;
					testParams.coilResistTestPer = newTestParamsPacket.data.coilResistTestPer;
					testParams.numFail = newTestParamsPacket.data.numFail;

					if (testParams.bounceTestPer > 1) {
						bounceTest = 1;
						bounceTimer = testParams.bounceTestPer;
					}
					if (testParams.coilCurrentTestPer > 1) {
						currentTest = 1;
						currentTimer = testParams.coilCurrentTestPer;
					}
					if (testParams.coilResistTestPer > 1) {
						resistTest = 1;
						resistTimer = testParams.coilResistTestPer;
					}

				}
			}
		}

		if (sendResponse == 1) {
			for (;;) {
				uint8_t status = USBD_TEMPLATE_Transmit(&hUsbDeviceFS, usbBuffer,
						count);
				if (status == USBD_OK)
					break;
				osDelay(1);
			}
		}
	}
	/* USER CODE END StartProcessUSB */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM14) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
