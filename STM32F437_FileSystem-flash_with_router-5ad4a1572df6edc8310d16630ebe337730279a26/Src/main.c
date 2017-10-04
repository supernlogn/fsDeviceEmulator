/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include <user_s25fl.h>

/* USER CODE BEGIN Includes */
#include <boot_data.h>
#ifdef HANDLE_FIRST_BOOT
boot_data_t boot_data __attribute__ ((section(".eeprom"))) = {
		.is_first_boot = 1,
		.flash_needs_cleaning = {1,1},

};
#endif

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
#include <string.h>
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
 {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_DeInit();
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
  MX_SPI4_Init();

  /* USER CODE BEGIN 2 */
  /* intialize flash driver & select device */
//  status = s25fl_io.s25fl_io_init(&s25fl_io, &hspi4);

  /* USER CODE END 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //////p. 215 -- > 2048
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // TODO: specify this!!
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // TODO : Ok @RCC_HCLK_DIV2

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI4 init function */
static void MX_SPI4_Init(void)
{

  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* Buffers for writing to files*/

unsigned char data0Buffer[2048]; //  * 128 --> 256KB DISK0
unsigned char data1Buffer[1024];  // * 128 --> 128KB DISK1


/* Buffers for checking written files*/

unsigned char data0Buffer1[2048];
unsigned char data0Buffer2[2048];
unsigned char data0Buffer3[2048];
unsigned char data0Buffer4[2048];
unsigned char data0Buffer5[2048];
unsigned char data0Buffer6[2048];
unsigned char data0Buffer7[2048];
unsigned char data0Buffer8[2048];


unsigned char data1Buffer1[1024];
unsigned char data1Buffer2[1024];
unsigned char data1Buffer3[1024];
unsigned char data1Buffer4[1024];
unsigned char data1Buffer5[1024];
unsigned char data1Buffer6[1024];
unsigned char data1Buffer7[1024];
unsigned char data1Buffer8[1024];


/* USER CODE END 4 */

FATFS FatFs;   /* Work area (filesystem object) for logical drive */

/* Files and File Paths */

FIL f0p1;
FIL f0p2;
FIL f0p3;
FIL f0p4;
FIL f0p5;
FIL f0p6;
FIL f0p7;
FIL f0p8;

FIL f1p1;
FIL f1p2;
FIL f1p3;
FIL f1p4;
FIL f1p5;
FIL f1p6;
FIL f1p7;
FIL f1p8;

char file_0path1[100];
char file_0path2[100];
char file_0path3[100];
char file_0path4[100];
char file_0path5[100];
char file_0path6[100];
char file_0path7[100];
char file_0path8[100];

char file_1path1[100];
char file_1path2[100];
char file_1path3[100];
char file_1path4[100];
char file_1path5[100];
char file_1path6[100];
char file_1path7[100];
char file_1path8[100];


/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  volatile int i;
  int j;
  volatile int choice = 0;
  volatile FRESULT fres = FR_OK;

  for(i = 0; i < 2048; ++i)
	  {
	    data0Buffer[i] = 0xAA;

	    data0Buffer1[i] = 0;
		data0Buffer2[i] = 0;
		data0Buffer3[i] = 0;
		data0Buffer4[i] = 0;
		data0Buffer5[i] = 0;
		data0Buffer6[i] = 0;
		data0Buffer7[i] = 0;
		data0Buffer8[i] = 0;

	  }

  for(i = 0; i < 1024; ++i)
	  {
	    data1Buffer[i] = 0x55;

	    data1Buffer1[i] = 0;
		data1Buffer2[i] = 0;
		data1Buffer3[i] = 0;
		data1Buffer4[i] = 0;
		data1Buffer5[i] = 0;
		data1Buffer6[i] = 0;
		data1Buffer7[i] = 0;
		data1Buffer8[i] = 0;
	  }


  user_s25fl_driver_init();

//     To check the read writing speed limits without a file system!!!
//
//    	for(i = 0; i < 25; ++i) {
//    	disk_write(0, dataBuffer1, i,1);
//    	disk_read(0, dataBuffer2, i,1);
//    	for(j = 0; j< 2048; ++j) {
//    		if(dataBuffer1[j] != dataBuffer2[j]) {
//    			ans = 1;
//    			break;
//    		}
//    	}
//    	if(ans == 1) {
//    		break;
//    	}
//    }
//    for(i = 0; i < 25; ++i) {
//    	disk_write(1, dataBuffer3, i,1);
//    	disk_read(1, dataBuffer4, i,1);
//    	for(j = 0; j< 2048; ++j) {
//    		if(dataBuffer1[j] != dataBuffer2[j]) {
//    			ans02 = 1;
//    			break;
//    		}
//    	}
//    	if(ans02 == 1) {
//    		break;
//    	}
//    }


  /* init code for FATFS */

  MX_FATFS_Init();
  disk_initialize(0);
  disk_initialize(1); //TODO : only 1 working disk
  s25fl_ios[0].force_synchronous = 1;
  s25fl_ios[1].force_synchronous = 1; //TODO : only 1 working disk

  /* USER CODE BEGIN 5 */

    volatile int ans01 = 0;
	volatile int ans02 = 0;
	volatile int ans03 = 0;
	volatile int ans04 = 0;
	volatile int ans05 = 0;
	volatile int ans06 = 0;
	volatile int ans07 = 0;
	volatile int ans08 = 0;

	volatile int ans11 = 0;
	volatile int ans12 = 0;
	volatile int ans13 = 0;
	volatile int ans14 = 0;
	volatile int ans15 = 0;
	volatile int ans16 = 0;
	volatile int ans17 = 0;
	volatile int ans18 = 0;


  /* test file system USER_Path[0] */

  fres = f_mount(&FatFs, USER_Path[0], 1);
  fres = f_mkfs(USER_Path[0], 1, (BLOCK_SIZE));
  f_mount(NULL, USER_Path[0], 1);
  fres = f_mount(&FatFs, USER_Path[0], 1);

  sprintf(file_0path1,"%smy0data1.txt",USER_Path[0]);
  sprintf(file_0path2,"%smy0data2.txt",USER_Path[0]);
  sprintf(file_0path3,"%smy0data3.txt",USER_Path[0]);
  sprintf(file_0path4,"%smy0data4.txt",USER_Path[0]);
  sprintf(file_0path5,"%smy0data5.txt",USER_Path[0]);
  sprintf(file_0path6,"%smy0data6.txt",USER_Path[0]);
  sprintf(file_0path7,"%smy0data7.txt",USER_Path[0]);
  sprintf(file_0path8,"%smy0data8.txt",USER_Path[0]);

  /* test file system USER_Path[1] */
  fres = f_mount(&FatFs, USER_Path[1], 1);
  fres = f_mkfs(USER_Path[1], 1, (BLOCK_SIZE));
  f_mount(NULL, USER_Path[1], 1);
  fres = f_mount(&FatFs, USER_Path[1], 1);
  //fres = f_mkdir("sub1");

  sprintf(file_1path1,"%smy1data.txt",USER_Path[1]);
  sprintf(file_1path2,"%smy1data2.txt",USER_Path[1]);
  sprintf(file_1path3,"%smy1data3.txt",USER_Path[1]);
  sprintf(file_1path4,"%smy1data4.txt",USER_Path[1]);
  sprintf(file_1path5,"%smy1data5.txt",USER_Path[1]);
  sprintf(file_1path6,"%smy1data6.txt",USER_Path[1]);
  sprintf(file_1path7,"%smy1data7.txt",USER_Path[1]);
  sprintf(file_1path8,"%smy1data8.txt",USER_Path[1]);


  UINT bytes_written, bytes_read;

  for(;;)
  {
    osDelay(1);
    switch(choice)
    {
	case 0:
		fres = f_open(&f0p1, file_0path1, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f0p1, data0Buffer, sizeof(data0Buffer), &bytes_written);
		}
		fres = f_close(&f0p1);
		choice = 1;//to continue the process
		break;
	case 1:
		fres = f_open(&f0p1,file_0path1, FA_READ | FA_OPEN_EXISTING);
		ans01 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f0p1, data0Buffer1, sizeof(data0Buffer1), &bytes_read);
			for(j = 0; j< 2048; ++j) {
				if(data0Buffer[j] != data0Buffer1[j]) {
					ans01 = 1;
					break;
			}
		}
		if(ans01 == 1) {
			break;
		}
		}
		fres = f_close(&f0p1); //two files open!
		choice = 2;//to continue the process
		break;
	case 2:
		disk_read(0,data0Buffer1,i,1);
		choice = 3;//to continue the process
		break;
	case 3:
		fres = f_open(&f0p2, file_0path2, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f0p2, data0Buffer, sizeof(data0Buffer), &bytes_written);
		}
		fres = f_close(&f0p2);
		choice = 4;//to continue the process
		break;
    case 4:
		fres = f_open(&f0p2,file_0path2, FA_READ | FA_OPEN_EXISTING);
		ans02 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
		fres = f_read(&f0p2, data0Buffer2, sizeof(data0Buffer2), &bytes_read);
			for(j = 0; j< 2048; ++j) {
				if(data0Buffer[j] != data0Buffer2[j]) {
					ans02 = 1;
					break;
			}
		}
		if(ans02 == 1) {
			break;
		}
		}
		fres = f_close(&f0p2); //two files open!
		choice = 5;//to continue the process
		break;
    case 5:
    	disk_read(0,data0Buffer2,i,1);
    	choice = 6;//to continue the process
    	break;
    case 6:
		fres = f_open(&f0p3, file_0path3, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f0p3, data0Buffer, sizeof(data0Buffer), &bytes_written);
		}
		fres = f_close(&f0p3);
		choice = 7;//to continue the process
		break;
    case 7:
		fres = f_open(&f0p3,file_0path3, FA_READ | FA_OPEN_EXISTING);
		ans03 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f0p3, data0Buffer3, sizeof(data0Buffer3), &bytes_read);
			for(j = 0; j< 2048; ++j) {
				if(data0Buffer[j] != data0Buffer3[j]) {
					ans03 = 1;
					break;
			}
		}
		if(ans03 == 1) {
			break;
		}
		}
		fres = f_close(&f0p3); //two files open!
		choice = 8;//to continue the process
		break;
    case 8:
    	disk_read(0,data0Buffer3,i,1);
    	choice = 9;//to continue the process
    	break;
    case 9:
		fres = f_open(&f0p4, file_0path4, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f0p4, data0Buffer, sizeof(data0Buffer), &bytes_written);
		}
		fres = f_close(&f0p4);
		choice = 10;//to continue the process
		break;
    case 10:
		fres = f_open(&f0p4,file_0path4, FA_READ | FA_OPEN_EXISTING);
		ans04 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f0p4, data0Buffer4, sizeof(data0Buffer4), &bytes_read);
			for(j = 0; j< 2048; ++j) {
				if(data0Buffer[j] != data0Buffer4[j]) {
					ans04 = 1;
					break;
			}
		}
		if(ans04 == 1) {
			break;
		}
		}
		fres = f_close(&f0p4); //two files open!
		choice = 11;//to continue the process
		break;
    case 11:
		disk_read(0,data0Buffer4,i,1);
		choice = 12;//to continue the process
		break;


	case 12:
		fres = f_open(&f0p5, file_0path5, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f0p5, data0Buffer, sizeof(data0Buffer), &bytes_written);
		}
		fres = f_close(&f0p5);
		choice = 13;//to continue the process
		break;
	case 13:
		fres = f_open(&f0p5,file_0path5, FA_READ | FA_OPEN_EXISTING);
		ans05 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f0p5, data0Buffer5, sizeof(data0Buffer5), &bytes_read);
			for(j = 0; j< 2048; ++j) {
				if(data0Buffer[j] != data0Buffer5[j]) {
					ans05 = 1;
					break;
			}
		}
		if(ans05 == 1) {
			break;
		}
		}
		fres = f_close(&f0p5); //two files open!
		choice = 14;//to continue the process
		break;
	case 14:
		disk_read(0,data0Buffer5,i,1);
		choice = 15;//to continue the process
		break;
	case 15:
		fres = f_open(&f0p6, file_0path6, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f0p6, data0Buffer, sizeof(data0Buffer), &bytes_written);
		}
		fres = f_close(&f0p6);
		choice = 16;//to continue the process
		break;
    case 16:
		fres = f_open(&f0p6,file_0path6, FA_READ | FA_OPEN_EXISTING);
		ans06 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f0p6, data0Buffer6, sizeof(data0Buffer6), &bytes_read);
			for(j = 0; j< 2048; ++j) {
				if(data0Buffer[j] != data0Buffer6[j]) {
					ans06 = 1;
					break;
			}
		}
		if(ans06 == 1) {
			break;
		}
		}
		fres = f_close(&f0p6); //two files open!
		choice = 17;//to continue the process
		break;
    case 17:
    	disk_read(0,data0Buffer6,i,1);
    	choice = 18;//to continue the process
    	break;
    case 18:
		fres = f_open(&f0p7, file_0path7, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f0p7, data0Buffer, sizeof(data0Buffer), &bytes_written);
		}
		fres = f_close(&f0p7);
		choice = 19;//to continue the process
		break;
    case 19:
		fres = f_open(&f0p7,file_0path7, FA_READ | FA_OPEN_EXISTING);
		ans07 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f0p7, data0Buffer7, sizeof(data0Buffer7), &bytes_read);
			for(j = 0; j< 2048; ++j) {
				if(data0Buffer[j] != data0Buffer7[j]) {
					ans07 = 1;
					break;
			}
		}
		if(ans07 == 1) {
			break;
		}
		}
		fres = f_close(&f0p7); //two files open!
		choice = 20;//to continue the process
		break;
    case 20:
    	disk_read(0,data0Buffer7,i,1);
    	choice = 21;//to continue the process
    	break;
    case 21:
		fres = f_open(&f0p8, file_0path8, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f0p8, data0Buffer, sizeof(data0Buffer), &bytes_written);
		}
		fres = f_close(&f0p8);
		choice = 22;//to continue the process
		break;
    case 22:
		fres = f_open(&f0p8,file_0path8, FA_READ | FA_OPEN_EXISTING);
		ans08 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f0p8, data0Buffer8, sizeof(data0Buffer8), &bytes_read);
			for(j = 0; j< 2048; ++j) {
				if(data0Buffer[j] != data0Buffer8[j]) {
					ans08 = 1;
					break;
			}
		}
		if(ans08 == 1) {
			break;
		}
		}
		fres = f_close(&f0p8); //two files open!
		choice = 23;//to continue the process
		break;
    case 23:
		disk_read(0,data0Buffer8,i,1);
		choice = 30;//to continue the process
		break;

//change to disk 1 from casedata1Buffer

	case 30:
		fres = f_open(&f1p1, file_1path1, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f1p1, data1Buffer, sizeof(data1Buffer), &bytes_written);
		}
		fres = f_close(&f1p1);
		choice = 31;//to continue the process
		break;
	case 31:
		fres = f_open(&f1p1,file_1path1, FA_READ | FA_OPEN_EXISTING);
		ans11 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f1p1, data1Buffer1, sizeof(data1Buffer1), &bytes_read);
			for(j = 0; j< 1024; ++j) {
				if(data1Buffer[j] != data1Buffer1[j]) {
					ans11 = 1;
					break;
			}
		}
		if(ans11 == 1) {
			break;
		}
		}
		fres = f_close(&f1p1); //two files open!
		choice = 32;//to continue the process
		//choice = 33;
		break;
	case 32:
		disk_read(1,data1Buffer1,i,1);
		choice = 33;//to continue the process

		break;
	case 33:
		fres = f_open(&f1p2, file_1path2, FA_CREATE_ALWAYS | FA_WRITE);
		ans12=3; //..to be sure ---- checkpoint
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f1p2, data1Buffer, sizeof(data1Buffer), &bytes_written);
		}
		fres = f_close(&f1p2);
		choice = 34;//to continue the process
		break;
    case 34:
		fres = f_open(&f1p2,file_1path2, FA_READ | FA_OPEN_EXISTING);
		ans12 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f1p2, data1Buffer2, sizeof(data1Buffer2), &bytes_read);
			for(j = 0; j< 1024; ++j) {
				if(data1Buffer[j] != data1Buffer2[j]) {
					ans12 = 1;
					break;
			}
		}
		if(ans12 == 1) {
			break;
		}
		}
		fres = f_close(&f1p2); //two files open!
		choice = 35;//to continue the process//thema
		break;
    case 35:
    	//disk_read(1,data1Buffer2,i,1);//  ENTER WITH F5 ///only this disk read breaks ?????
    	choice = 36;//to continue the process
    	break;
    case 36:
		fres = f_open(&f1p3, file_1path3, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f1p3, data1Buffer, sizeof(data1Buffer), &bytes_written);
		}
		fres = f_close(&f1p3);
		choice = 37;//to continue the process
		break;
    case 37:
		fres = f_open(&f1p3,file_1path3, FA_READ | FA_OPEN_EXISTING);
		ans13 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f1p3, data1Buffer3, sizeof(data1Buffer3), &bytes_read);
			for(j = 0; j< 1024; ++j) {
				if(data1Buffer[j] != data1Buffer3[j]) {
					ans13 = 1;
				break;
			}
		}
		if(ans13 == 1) {
			break;
		}
		}
		fres = f_close(&f1p3); //two files open!
		//choice = 39; ok commented
		choice = 38;//to continue the process
		break;
    case 38:
    	disk_read(1,data1Buffer3,i,1);
    	choice = 39;//to continue the process
    	break;
    case 39:
		fres = f_open(&f1p4, file_1path4, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f1p4, data1Buffer, sizeof(data1Buffer), &bytes_written);
		}
		fres = f_close(&f1p4);
		choice = 40;//to continue the process
		break;
    case 40:
		fres = f_open(&f1p4,file_1path4, FA_READ | FA_OPEN_EXISTING);
		ans14 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f1p4, data1Buffer4, sizeof(data1Buffer4), &bytes_read);
			for(j = 0; j< 1024; ++j) {
				if(data1Buffer[j] != data1Buffer4[j]) {
					ans14 = 1;
					break;
			}
		}
		if(ans14 == 1) {
			break;
		}
		}
		fres = f_close(&f1p4); //two files open!
		choice = 41;//to continue the process ok commented
		//choice = 42;
		break;
    case 41:
		disk_read(1,data1Buffer4,i,1);
		choice = 42;//to continue the process
		break;
	case 42:
		fres = f_open(&f1p5, file_1path5, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f1p5, data1Buffer, sizeof(data1Buffer), &bytes_written);
		}
		fres = f_close(&f1p5);
		choice = 43;//to continue the process
		break;
	case 43:
		fres = f_open(&f1p5,file_1path5, FA_READ | FA_OPEN_EXISTING);
		ans15 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f1p5, data1Buffer5, sizeof(data1Buffer5), &bytes_read);
			for(j = 0; j< 1024; ++j) {
				if(data1Buffer[j] != data1Buffer5[j]) {
					ans15 = 1;
					break;
			}
		}
		if(ans15 == 1) {
			break;
		}
		}
		fres = f_close(&f1p5); //two files open!
		choice = 44;//to continue the process
		//choice = 45; ok commented
		break;
	case 44:
		disk_read(1,data1Buffer5,i,1);
		choice = 45;//to continue the process
		break;
	case 45:
		fres = f_open(&f1p6, file_1path6, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f1p6, data1Buffer, sizeof(data1Buffer), &bytes_written);
		}
		fres = f_close(&f1p6);
		choice = 46;//to continue the process
		break;
    case 46:
		fres = f_open(&f1p6,file_1path6, FA_READ | FA_OPEN_EXISTING);
		ans16 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f1p6, data1Buffer6, sizeof(data1Buffer6), &bytes_read);
			for(j = 0; j< 1024; ++j) {
				if(data1Buffer[j] != data1Buffer6[j]) {
					ans16 = 1;
					break;
			}
		}
		if(ans16 == 1) {
			break;
		}
		}
		fres = f_close(&f1p6); //two files open!
		choice = 47;//to continue the process
		//choice = 48; ??
		break;
    case 47:
    	disk_read(1,data1Buffer6,i,1);
    	choice = 48;//to continue the process
    	break;
    case 48:
		fres = f_open(&f1p7, file_1path7, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		  fres = f_write(&f1p7, data1Buffer, sizeof(data1Buffer), &bytes_written);
		}
		fres = f_close(&f1p7);
		choice = 49;//to continue the process
		break;
    case 49:
		fres = f_open(&f1p7,file_1path7, FA_READ | FA_OPEN_EXISTING);
		ans17 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f1p7, data1Buffer7, sizeof(data1Buffer7), &bytes_read);
			for(j = 0; j< 1024; ++j) {
				if(data1Buffer[j] != data1Buffer7[j]) {
					ans17 = 1;
					break;
			}
		}
		if(ans17 == 1) {
			break;
		}
		}
		fres = f_close(&f1p7); //two files open!
		choice = 50;//to continue the process ??
		//choice = 51;
		break;
    case 50:
    	disk_read(1,data1Buffer7,i,1);
    	choice = 51;//to continue the process
    	break;
    case 51:
		fres = f_open(&f1p8, file_1path8, FA_CREATE_ALWAYS | FA_WRITE);
		for(i = 0; i < 128; ++i) {
		fres = f_write(&f1p8, data1Buffer, sizeof(data1Buffer), &bytes_written);
		}
		fres = f_close(&f1p8);
		choice = 52;//to continue the process
		break;
    case 52:
		fres = f_open(&f1p8,file_1path8, FA_READ | FA_OPEN_EXISTING);
		ans18 = 2; // to know where I am
		for(i = 0; i < 128; ++i) {
			fres = f_read(&f1p8, data1Buffer8, sizeof(data1Buffer8), &bytes_read);
			for(j = 0; j< 1024; ++j) {
				if(data1Buffer[j] != data1Buffer8[j]) {
					ans18 = 1;
					break;
			}
		}
		if(ans18 == 1) {
			break;
		}
		}
		fres = f_close(&f1p8); //two files open!
		choice = 54;//to continue the process
		//choice = 55; ???
		break;
    case 53:
		disk_read(1,data1Buffer8,i,1);
		choice = 0;//to restart !!!!! :)
		break;
    default:
    	break;

    }
  }



  /* USER CODE END 5 */

  for(;;)
  {
	  osDelay(1);
  }
} // end start default task




/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
