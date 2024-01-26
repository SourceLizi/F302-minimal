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
  Date        Author        Notes
  2023/05/05 SourceLizi  Initial release
  2024/01/26 SourceLizi  Fix the bug that data was read when QMI8658C not ready, update imu gyro bias
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_eskf.h"
#include "usbd_cdc_if.h"
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
 SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
//static uint8_t rx, tx, addr = 0x01;
uint8_t upload_flag = 0;
uint8_t first_data = 0;
//uint16_t tx_16,rx_16;
extern USBD_HandleTypeDef hUsbDeviceFS;

#define IMU_PI 3.14159265358979f

typedef struct {
	int16_t acc[3];
	int16_t gyro[3];
}gyro_acc_data_t;

typedef __packed struct{
	uint16_t header;
	uint8_t func;
	uint8_t len;
	
	int16_t ROL;
	int16_t PIT;
	int16_t YAW;
	uint16_t dummy1;
	uint32_t dummy2;
	
	uint8_t sum;
}packet_t;

uint32_t timetamp,last_timetamp = 0;
uint32_t delta_t;

uint8_t spi_byte;


packet_t upload_packet;
gyro_acc_data_t imu_data;
imu_data_f32_t imu_data_f32;
euler_t eular;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void spi_read_bytes(uint8_t addr, uint8_t* rx_data,uint16_t size){
//		while((SPI2->SR & SPI_SR_BSY)==SPI_SR_BSY); 
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);//CS low
//	 
//		SPI2->DR = addr | 0x80;
//		for(int i=0;i < size; i++){
//			while((SPI2->SR & SPI_SR_BSY)==SPI_SR_BSY);
//			rx_data[i] =  SPI2->DR;
//		}
//		
//		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == GPIO_PIN_SET);
//		//HAL_SPI_TransmitReceive(&hspi2,&tx,&rx,1,100);

//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);//CS high
//}

//void spi_write_bytes(uint8_t addr, uint8_t* tx_data,uint16_t size){
//	while((SPI2->SR & SPI_SR_BSY)==SPI_SR_BSY);
//	
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);//CS low  
//		SPI2->DR = addr & 0x7F;
//		for(int i=0;i < size; i++){
//			while((SPI2->SR & SPI_SR_BSY)==SPI_SR_BSY);
//			SPI2->DR = tx_data[i];
//		}
//		while((SPI2->SR & SPI_SR_BSY)==SPI_SR_BSY);  
//		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == GPIO_PIN_SET);
//		//HAL_SPI_TransmitReceive(&hspi2,&tx,&rx,1,100);

//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);//CS high
//}

void spi_write_reg(uint8_t addr, uint8_t tx_byte){
	uint16_t rx_data, tx_data;
	tx_data  = ((uint16_t)(addr & 0x7F) << 8) | (uint16_t)tx_byte;
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t*)&tx_data,(uint8_t*)&rx_data,1,100);
}

uint8_t spi_read_reg(uint8_t addr){
	uint16_t rx_data, tx_data = 0;
	tx_data  = (uint16_t)(addr | 0x80) << 8;
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t*)&tx_data,(uint8_t*)&rx_data,1,100);
	return (uint8_t)(rx_data & 0xFF);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	upload_flag = 1;
}

uint8_t data_to_send[16];
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	int16_t _temp;
	int32_t _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	CDC_Transmit_FS((uint8_t*)data_to_send, _cnt);
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
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//__HAL_SPI_ENABLE(&hspi2);
	
	HAL_Delay(2000);
//	tx = 0x01;
//	spi_write_bytes(0x60,&tx,1);
//	HAL_Delay(20);
//	
//	tx = 0x7F;
//	spi_write_bytes(0x08,&tx,1);
//	HAL_Delay(80);
	spi_write_reg(0x60, 0x01);
	HAL_Delay(30);
	spi_write_reg(0x02,0x60);
	spi_write_reg(0x08,0x03);
	spi_write_reg(0x03,0x22); //+-8g , 2khz
	spi_write_reg(0x04, 0x72); //2048dps, 2khz
	//spi_write_reg(0x06, 0x55);
	imu_eskf_init();
	upload_packet.header = 0xAAAA;
	upload_packet.func = 0x01;
	upload_packet.len = 12;
	
	HAL_Delay(20);
	
	HAL_TIM_Base_Start_IT(&htim16); //10Hz
	
	last_timetamp = 0;
	TIM17->CNT = 0;
	HAL_TIM_Base_Start(&htim17); 
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//spi_read_bytes(addr,&rx,1);
		spi_byte = spi_read_reg(0x2E); //STATUS0
		if((spi_byte & 0x03) == 0x03){
				timetamp = TIM17->CNT;
				for(int i=0;i<12;i++){
						((uint8_t*)&imu_data)[i] = spi_read_reg(0x35+i);
				}
		//		for(int i=0;i<3;i++){
		//				((uint8_t*)&timetamp)[i] = spi_read_reg(0x30+i);
		//		}
				if(timetamp < last_timetamp){
					delta_t = (65536 - last_timetamp) + timetamp;
				}else{
					delta_t = timetamp - last_timetamp;
				}
				last_timetamp = timetamp;
				
				imu_data_f32.ax = imu_data.acc[0] / 32768.0f * 8.0f;
				imu_data_f32.ay = imu_data.acc[1] / 32768.0f * 8.0f;
				imu_data_f32.az = imu_data.acc[2] / 32768.0f * 8.0f;
				
				imu_data_f32.gx = (imu_data.gyro[0]+133) / 32768.0f * 2048 /180*IMU_PI ;
				imu_data_f32.gy = (imu_data.gyro[1]+31) / 32768.0f * 2048/180*IMU_PI;
				imu_data_f32.gz = (imu_data.gyro[2]+11) / 32768.0f * 2048/180*IMU_PI;
				
				imu_eskf_update(&imu_data_f32, ((float)delta_t)/1000000.0f);
				imu_eskf_eular(&eular);
				
				if(upload_flag){
					upload_flag = 0;
					if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
						ANO_DT_Send_Status(-eular.roll, eular.pitch, -eular.yaw, 0,0,0);
						HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					}else{
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
					}
				}
			}
//		
//		//spi_read_bytes(0x3B,(uint8_t*)gyro_data,6);
//		
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		//HAL_Delay(1);
		
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_TIM16
                              |RCC_PERIPHCLK_TIM17;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 720-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 5000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 72-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

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
