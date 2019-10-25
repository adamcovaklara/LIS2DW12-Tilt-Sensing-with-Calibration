/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "motion_tl.h"
#include "MotionTL_Manager.h"
#include "com.h"
#include "stm32l4xx_nucleo.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_exti.h"
#include "stm32l4xx_nucleo.h"
#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
#include "app_mems_int_pin_a_interface.h"
	
#ifndef PI
#define PI (3.141592653589793)
#endif

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  Handle DEMO State Machine
 */
typedef enum
{
  STATUS_IDLE,
  STATUS_SET_FIFO_MODE,
  STATUS_FIFO_RUN,
  STATUS_FIFO_DOWNLOAD,
  STATUS_SET_FIFO_BYPASS_MODE
} DEMO_FIFO_STATUS_t;

/* Private define ------------------------------------------------------------*/
#define FIFO_Interrupt_Pin GPIO_PIN_3
#define FIFO_Interrupt_GPIO_Port GPIOA  
#define FIFO_WATERMARK    10 /*!< FIFO size limit */
#define ACCELEROMETER_SENSOR                    0x00000010U
#define MAX_BUF_SIZE 256
#define FIFO_INTERRUPT    FIFO_INTERRUPT_THRESHOLD /*!< Chosen FIFO INTERRUPT event type */
#define LIS2DW12_SAMPLE_ODR         50.0f /*!< Sample Output Data Rate [Hz] */
#define LIS2DW12_FULL_SCALE					2
#define ENABLE  1 /*!< Enable LIS2DW12 FIFO functions */
#define PIN  PWR_PDCRA_PA3 /*!< PIN with FIFO Interrupt */

/* Variables ---------------------------------------------------------*/
//static char dataOut[MAX_BUF_SIZE];
static uint8_t fifo_flag = 0;
volatile uint8_t MemsEventDetected = 0;
static DEMO_FIFO_STATUS_t DemoFifoStatus = STATUS_SET_FIFO_MODE;
static LIS2DW12_Object_t LIS2DW12;
int ret;
struct parameters { 
		float offset_x, offset_y, offset_z, gain_x, gain_y, gain_z;
};
struct parameters Parameters;

/* Function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_IKS01A3_LIS2DW12_FIFOMode_Process(void);
static int32_t LIS2DW12_FIFO_Set_Bypass_Mode(void);
static int32_t LIS2DW12_FIFO_Set_FIFO_Mode(void);
static float LIS2DW12_Read_All_FIFO_Data(void);
static void MX_TIM_ALGO_Init(void);

/* These "redundant" lines are here to fulfil MISRA C-2012 rule 8.4 */
extern volatile uint8_t FlashEraseRequest;
extern volatile uint32_t SensorsEnabled;
volatile uint8_t FlashEraseRequest = 0;
volatile uint32_t SensorsEnabled = 0;

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : FIFO_Interrupt_Pin */
  GPIO_InitStruct.Pin = FIFO_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(FIFO_Interrupt_GPIO_Port, &GPIO_InitStruct);
}

void MX_USART2_UART_Init(void)
{
 huart2.Instance = USART2;
 huart2.Init.BaudRate = 115200;
 huart2.Init.WordLength = UART_WORDLENGTH_8B;
 huart2.Init.StopBits = UART_STOPBITS_1;
 huart2.Init.Parity = UART_PARITY_NONE;
 huart2.Init.Mode = UART_MODE_TX_RX;
 huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 huart2.Init.OverSampling = UART_OVERSAMPLING_16;
 huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
 huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
 HAL_UART_Init(&huart2);
} 

static void Init_Sensors(void)
{
  (void)IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2DW12_0, MOTION_ACCELERO);

  /* Set accelerometer:
   *   - ODR >= 50 Hz
   *   - FS   = <-2g, 2g>
   */
  
	(void)IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, LIS2DW12_SAMPLE_ODR);
  (void)IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, LIS2DW12_FULL_SCALE);
}

static void MX_CRC_Init(void)
{
  __CRC_CLK_ENABLE();
}

void initLIS2DW12(void)
{
  LIS2DW12_IO_t            io_ctx;

  /* Configure the accelero driver */
  io_ctx.BusType     = LIS2DW12_I2C_BUS; /* I2C */
  io_ctx.Address     = LIS2DW12_I2C_ADD_H;
  io_ctx.Init        = IKS01A3_I2C_Init;
  io_ctx.DeInit      = IKS01A3_I2C_DeInit;
  io_ctx.ReadReg     = IKS01A3_I2C_ReadReg;
  io_ctx.WriteReg    = IKS01A3_I2C_WriteReg;
  io_ctx.GetTick     = IKS01A3_GetTick;

  LIS2DW12_RegisterBusIO(&LIS2DW12, &io_ctx);
  
  // use high-performance mode (14-bit resolution)
  lis2dw12_power_mode_set(&(LIS2DW12.Ctx), LIS2DW12_HIGH_PERFORMANCE);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  TIM_ALGO init function
 * @param  None
 * @retval None
 * @details This function intializes the Timer used to synchronize the algorithm
 */
static void MX_TIM_ALGO_Init(void)
{
#if (defined (USE_STM32F4XX_NUCLEO))
#define CPU_CLOCK  84000000U

#elif (defined (USE_STM32L0XX_NUCLEO))
#define CPU_CLOCK  32000000U

#elif (defined (USE_STM32L1XX_NUCLEO))
#define CPU_CLOCK  32000000U

#elif (defined (USE_STM32L4XX_NUCLEO))
#define CPU_CLOCK  80000000U

#else
#error Not supported platform
#endif
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  // report the HAL error return state 
  printf("\r\nError.\r\n");
}

/**
  * @brief  Configure FIFO
  * @retval BSP status
  */
static int32_t LIS2DW12_FIFO_Demo_Config(void)
{
  lis2dw12_ctx_t *ctx = &(LIS2DW12.Ctx);
  /* Set FIFO watermark */
  if ((ret = lis2dw12_fifo_watermark_set(ctx, FIFO_WATERMARK)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}

static void Init_all(void)
{
    /* STM32xxxx HAL library initialization:
   *   - Configure the Flash prefetch, instruction and Data caches
   *   - Configure the Systick to generate an interrupt each 1 msec
   *   - Set NVIC Group Priority to 4
   *   - Global MSP (MCU Support Package) initialization
   */
  (void)HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
	
	/* Init GPIO */
  MX_GPIO_Init();
	
	/* Init CRC */
  MX_CRC_Init();
	
  /* Timer for algorithm synchronization initialization */
  MX_TIM_ALGO_Init();
  
  /* Initialize (disabled) Sensors */
  set_mems_int_pin_a_exti();
  Init_Sensors();
  
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  /* TiltSensing API initialization function */
  MotionTL_manager_init();

  /* Initialize all configured peripherals */
  initLIS2DW12();
  if (LIS2DW12_FIFO_Demo_Config() != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
	
	MX_USART2_UART_Init();
  
  printf("\r\nInit done.\r\n");
}


/**
  * @brief  Process of the LIS2DW12 FIFO Mode application
  * @retval None
  */
void MX_IKS01A3_LIS2DW12_FIFOMode_Process(void)
{
  /* Reset Interrupt flag */
  fifo_flag = 0;

  /* Change this variable only if DemoFifoStatus is STATUS_IDLE */
  if (DemoFifoStatus == STATUS_IDLE)
  {
    DemoFifoStatus = STATUS_SET_FIFO_MODE;
  }

  /* Handle DEMO State Machine */
  switch (DemoFifoStatus)
  {
    case STATUS_IDLE:
      break;

    case STATUS_SET_FIFO_MODE:
      if (LIS2DW12_FIFO_Set_FIFO_Mode() != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      DemoFifoStatus = STATUS_FIFO_RUN;
      break;

    case STATUS_FIFO_RUN:
      if (lis2dw12_fifo_wtm_flag_get(&(LIS2DW12.Ctx), &fifo_flag) != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      if (fifo_flag == 1)
      {
          DemoFifoStatus = STATUS_FIFO_DOWNLOAD;
      }
      break;

    case STATUS_FIFO_DOWNLOAD:
        if (LIS2DW12_Read_All_FIFO_Data() != BSP_ERROR_NONE)
        {
          Error_Handler();
        }
        DemoFifoStatus = STATUS_SET_FIFO_BYPASS_MODE;
      break;

    case STATUS_SET_FIFO_BYPASS_MODE:
      if (LIS2DW12_FIFO_Set_Bypass_Mode() != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      DemoFifoStatus = STATUS_IDLE;
      break;

    default:
      Error_Handler();
      break;
  }
}

/**
  * @brief  Set FIFO bypass mode
  * @retval BSP status
  */
static int32_t LIS2DW12_FIFO_Set_Bypass_Mode(void)
{
	
  if ((ret = LIS2DW12_FIFO_Set_Mode(&LIS2DW12, (uint8_t)LIS2DW12_BYPASS_MODE)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}

/**
  * @brief  Set FIFO to FIFO mode
  * @retval BSP status
  */
static int32_t LIS2DW12_FIFO_Set_FIFO_Mode(void)
{
  /* Set FIFO mode to FIFO */
  if ((ret = LIS2DW12_FIFO_Set_Mode(&LIS2DW12, (uint8_t)LIS2DW12_FIFO_MODE)) != BSP_ERROR_NONE)
  {
    printf("\r\nError.\r\n");
    return ret;
  }
  
  return ret;
}

/**
 * @brief  Collect accelerometer data
 * @param  cal_data Pointer to 2D array of calibration data cal_data[num_records][3]
 * @param  num_records Number of records to be taken (3 axes per record)
 * @retval 0  Ok
 * @retval 1  Accelerometer error
 */
uint8_t CollectData(float cal_data[][3], uint32_t num_records)
{
  if ((SensorsEnabled & ACCELEROMETER_SENSOR) != ACCELEROMETER_SENSOR)
  {
    return 1;
  }
	return 0;
}

void NoCalibration_6point(void)
{
	Parameters.offset_x = 0.107238;
	Parameters.offset_y = 0.116876;
	Parameters.offset_z = 0.090524;
	Parameters.gain_x = 0.246318;
	Parameters.gain_y = 0.244256;
	Parameters.gain_z = 0.24522;
}

typedef struct myData
{
	float x, y, z;
} myData;

myData GetMyData(char * output)
{
	printf("%s\r\nScanning...\r\n", output);
	HAL_Delay(5000);
	IKS01A3_MOTION_SENSOR_Axes_t raw_data;
	IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, &raw_data);
	myData res = {raw_data.x, raw_data.y, raw_data.z};
	//printf("x: = %f, y = %f, z = %f\r\n", res.x/1000.0f, res.y/1000.0f, res.z/1000.0f);
	//HAL_Delay(5000);
	return res;
}

void NoCalibration_MiniMax(void)
{
	float x_max = 0.365068;
	float x_min = -0.139763;
	float y_max = 0.362633;
	float y_min = -0.135713;
	float z_max = 0.33633;
	float z_min = -0.163589;
	
	float sensitivity = 0.0f;
	// sensitivity check
  LIS2DW12_ACC_GetSensitivity(&LIS2DW12, &sensitivity);
	
	// division by full scale +-2g, g = 1000 mg
	float one_G = 1000/4;
	// division by 14-bit format sensitivity for full scale +-2g
  one_G *= sensitivity;
	
	Parameters.offset_x = -one_G * (x_max + x_min)/(x_max - x_min);
	Parameters.offset_y = -one_G * (y_max + y_min)/(y_max - y_min);
	Parameters.offset_z = -one_G * (z_max + z_min)/(z_max - z_min);
	Parameters.gain_x = (one_G - Parameters.offset_x) / x_max;
	Parameters.gain_y = (one_G - Parameters.offset_y) / y_max;
	Parameters.gain_z = (one_G - Parameters.offset_z) / z_max;
}

void Calibration_6point(void)
{
	myData data = GetMyData("Place the board horizontally on the table.");
	Parameters.offset_x = data.x;
	Parameters.offset_y = data.y;
	Parameters.offset_z = data.z;
	
	data = GetMyData("Turn the board over.");
	Parameters.offset_x = (Parameters.offset_x + data.x) / 2;
	Parameters.offset_y = (Parameters.offset_y + data.y) / 2;
	Parameters.gain_z = fabs(Parameters.offset_z + data.z) / 2;
	Parameters.offset_z = (Parameters.offset_z + data.z) / 2;
	
	data = GetMyData("Place the board vertically to the table (USB up).");
	Parameters.gain_y = data.y;

	data = GetMyData("Turn the board over (USB down).");
	Parameters.gain_y = fabs(Parameters.gain_y + data.y) / 2;
	
	data = GetMyData("Place the board in front of you (looking on USB).");
	Parameters.gain_x = data.x;
			
	data = GetMyData("Rotate the board by 180 degrees (USB hidden).");
	Parameters.gain_x = fabs(Parameters.gain_x + data.x) / 2;
	
	printf("\r\nCalibration done.\r\n");
	//printf("offset x: %f, offset y: %f, offset z: %f, gain x: %f, gain y: %f, gain z: %f", Parameters.offset_x, Parameters.offset_y, Parameters.offset_z, Parameters.gain_x,
	// Parameters.gain_y, Parameters.gain_z);
	//HAL_Delay(10000);
}

void Calibration_MiniMax(void)
{
	int wait = 10000;
	int tickstart = HAL_GetTick();

	myData data = GetMyData("Rotate the board in all directions for 10 seconds.");

	float x_min = data.x;
	float y_min = data.y;
	float z_min = data.z;
	float x_max = data.x;
	float y_max = data.y;
	float z_max = data.z;
	
	while ((HAL_GetTick() - tickstart) < wait)
	{
	data = GetMyData("");
	if (data.x < x_min)
		x_min = data.x;
	else if (data.x > x_max)
		x_max = data.x;
	if (data.y < y_min)
		y_min = data.y;
	else if (data.y > y_max)
		y_max = data.y;
	if (data.z < z_min)
		z_min = data.z;
	else if (data.z > z_max)
		z_max = data.z;
	}
	
	float sensitivity = 0.0f;
	// sensitivity check
  LIS2DW12_ACC_GetSensitivity(&LIS2DW12, &sensitivity);
	
	// division by full scale +-2g, g = 1000 mg
	float one_G = 1000/4;
	// division by 14-bit format sensitivity for full scale +-2g
  one_G *= sensitivity;
	
	Parameters.offset_x = -one_G * (x_max + x_min)/(x_max - x_min);
	Parameters.offset_y = -one_G * (y_max + y_min)/(y_max - y_min);
	Parameters.offset_z = -one_G * (z_max + z_min)/(z_max - z_min);
	Parameters.gain_x = (one_G - Parameters.offset_x) / x_max;
	Parameters.gain_y = (one_G - Parameters.offset_y) / y_max;
	Parameters.gain_z = (one_G - Parameters.offset_z) / z_max;
	
	printf("\r\nCalibration done.\r\n");
	//printf("offset x: %f, offset y: %f, offset z: %f, gain x: %f, gain y: %f, gain z: %f", Parameters.offset_x, Parameters.offset_y, Parameters.offset_z, Parameters.gain_x,
	//Parameters.gain_y, Parameters.gain_z);
	//HAL_Delay(10000);
}

/**
  * @brief  Read all unread FIFO data in cycle
  * @retval BSP status
  */
static float LIS2DW12_Read_All_FIFO_Data(void)
{
  uint8_t samples_to_read = 0;
  uint8_t i;
	
	float projX_cal = 0, projY_cal = 0, projZ_cal = 0;
 
	IKS01A3_MOTION_SENSOR_Axes_t raw_data;
  
  /* Get num of unread FIFO samples before reading data */
  if ((ret = lis2dw12_fifo_data_level_get(&(LIS2DW12.Ctx), &samples_to_read)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  for (i = 0; i < samples_to_read; ++i)
  {
    if (IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, &raw_data) != BSP_ERROR_NONE)
    {
      return ret;
    }
	
		// interpreting data as 14(6)-bit 2-complement		
		projX_cal += (int16_t)raw_data.x*Parameters.gain_x + Parameters.offset_x;
    projY_cal += (int16_t)raw_data.y*Parameters.gain_y + Parameters.offset_y;
    projZ_cal += (int16_t)raw_data.z*Parameters.gain_z + Parameters.offset_z;
  }

	float angleXZ_cal = atan2(projX_cal, projZ_cal) * 180 / PI;
	
	// normalize angle to interval (-180, 180)
	float angle_cal = fmod(angleXZ_cal, 180.0f);
	
	// convert angle to its acute representation
	if (angle_cal < 0)
		angle_cal = -angle_cal;
	if(angle_cal > 90)
		angle_cal = 180 - angle_cal;
	
	printf("\r\n\tEstimate of the angle: %.2f\r\n\r\n", angle_cal);
	
  return ret;
}

void uartPrint(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
} 

void uartPrintln(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
 char newline[2] = "\r\n";
 HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
} 

char getUserInput(UART_HandleTypeDef *huart, char * welcomeMSG, char * validOptions)
{
	uint8_t tmp = 0, res = 0;
	unsigned nOptions = strlen(validOptions), i;
	
	printf("%s\r\n", welcomeMSG);
	while (42) 
	{
			if (HAL_UART_Receive(huart, &tmp, 1, 1000) == HAL_OK)
			{
					if (tmp == 13)
					{
						for (i = 0; i < nOptions; ++i)
						{
							if (res == validOptions[i]) break;
						}
						if (i == nOptions)
						{
							printf("Error: invalid choice: %c\r\n", (char)res);
							continue;
						}
						return res;
					}
					res = tmp;
			}
	}
}

int main(void)
{ 	
  Init_all();
	
	char calib_type = getUserInput(&huart2, "For minimax type 'm', for 6-point 'p'.", "mp");
printf("Value: %c\r\n", calib_type);

char calibration = getUserInput(&huart2, "User calibration? Type y/n...", "yn");
printf("Value: %c\r\n", calibration);
 
	if (calibration == 'n' && calib_type == 'p')
		NoCalibration_6point();
	else if (calibration == 'n' && calib_type == 'm')
		NoCalibration_MiniMax();
	else if (calibration == 'y' && calib_type == 'p')
		Calibration_6point();
	else if (calibration == 'y' && calib_type == 'm')
		Calibration_MiniMax();
	
  /* Infinite loop */
  while (1)
  {
    MX_IKS01A3_LIS2DW12_FIFOMode_Process();
  }
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
