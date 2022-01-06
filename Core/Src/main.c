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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <strings.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	KHONG_DUOC_NHAN,
	DUOC_NHAN
}button_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int global = 10;
int cnt = 0;
uint8_t dir = 1;
uint16_t T = 0;
int16_t X_axis = 0;
int16_t Y_axis = 0;
int16_t Z_axis = 0;
float adc_vin;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void MY_Handler();

void TIM1_Init()
{
	__HAL_RCC_TIM1_CLK_ENABLE();
	uint16_t* PSC = (uint16_t*)0x40010028;
	uint16_t* ARR = (uint16_t*)0x4001002c;
	uint16_t* CNT = (uint16_t*)0x40010024;
	uint32_t* CR1 = (uint32_t*)0x40010000;
	uint32_t* SR  = (uint32_t*)0x40010010;
	uint32_t* DIER =(uint32_t*)0x4001000c;

	*ARR = 9;
	*PSC = 15999;
//	*DIER |= 1;		//de timer 1 tao ra su kien ngat --> gui --> nvic
	*CR1 |= 1;

}
void my_delay(int time_sec)
{
	uint32_t* SR  = (uint32_t*)0x40010010;
	for(int i = 0; i < time_sec; i++)
	{
		while(!(*SR & 1));
		*SR &= ~1;
	}
}

void my_time1_handle()
{
	uint32_t* SR  = (uint32_t*)0x40010010;
	*SR &= ~1;
}

void TIM1_UP_TIM10_IRQHandler()
{
	//application code
}

void TIM4_PWM_Init()
{
	//set up PD12 as TIMER4_CH1
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(0x40020c00);
	*GPIOD_MODER &= ~(0b11 << 24);
	*GPIOD_MODER |= (0b10 << 24);
	uint32_t* GPIOD_AFRH = (uint32_t*)(0x40020c24);
	*GPIOD_AFRH  |= (0b0010<<16);

	//set up time 4 f=1kHz
	//Ftimer = 16Mhz
	__HAL_RCC_TIM4_CLK_ENABLE();
	uint16_t* ARR = (uint16_t*)0x4000082c;
	uint16_t* PSC = (uint16_t*)0x40000828;
	uint32_t* CCR1 = (uint32_t*)0x40000834;

	*ARR = 99;			//~100%
	/*
	 * Ftimer = 16 000 000 ------> 1s
	 * 			16 000    <-----  1ms
	 * 			100       <-----  1ms
	 * */
	*PSC = 16000 / 100 - 1;

	*CCR1 = 25;

	uint16_t* CCMR1 = (uint16_t*)0x40000818;
	*CCMR1 &= ~(0b11);				// set 00: CC1 channel is configured as OUTPUT.
	*CCMR1 |= (0b110 << 4);				//PWM mode 1

	uint16_t* CCER = (uint16_t*)0x40000820;
	*CCER |= 1;

	uint32_t* CR1 = (uint32_t*)0x40000800;
	*CR1 |= 1;
}

void Modify_PWM(uint8_t value)
{
	uint32_t* CCR1 = (uint32_t*)0x40000834;
	*CCR1  = value;
}
void Tim1_Cap()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* MODER = (uint32_t*)0x40020000;
	*MODER &= ~(0b11<<16);
	*MODER |= (0b10<<16);

	uint32_t* AFRH = (uint32_t*)0x40020024;
	*AFRH |= 1;

	__HAL_RCC_TIM1_CLK_ENABLE();
	uint32_t* CR1 = (uint32_t*)0x40010000;
	uint16_t* ARR = (uint16_t*)0x4001002c;
	uint16_t* PSC = (uint16_t*)0x40010028;
	uint32_t* CCMR1 = (uint32_t*)0x40010018;
	uint32_t* CCER = (uint32_t*)0x40010020;
	uint32_t* SMCR = (uint32_t*)0x40010008;

	*SMCR |= 0b100;			// set RESET mode
	*SMCR |= 0b100 << 4;	// set TRIGGER as edge detector
	*ARR = 0xffff;			// set max value for ARR
	*PSC = 	16 -1;			// 1 cnt ~ 1us

	*CCMR1 |= 1;			// IC1 is mapped on TI1
	*CCER |= 1;				// enable capture/compare
	*CR1 |= 1;				// enable timer 1

}
uint16_t Get_T()
{
	uint16_t* CCR1 = (uint16_t*)0x40010034;
	return *CCR1 + 1;
}

#define TIMER5_BASE 0x40000c00
uint32_t* CNT = (uint32_t*)(TIMER5_BASE + 0x24);
uint32_t* CCR1 = (uint32_t*)(TIMER5_BASE + 0x34);
uint32_t* CCR2 = (uint32_t*)(TIMER5_BASE + 0x38);
uint16_t cnt_val = 0;
uint16_t cap1_val = 0;
uint16_t cap2_val = 0;
void time5_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* MODER = (uint32_t*)0x40020000;
	*MODER &= ~(0b11);
	*MODER |= (0b10);
	uint32_t* AFRL = (uint32_t*)0x40020020;
	*AFRL |= 2;

	__HAL_RCC_TIM5_CLK_ENABLE();
	//timer basic
	uint32_t* CR1 = (uint32_t*)(TIMER5_BASE);
	uint32_t* PSC = (uint32_t*)(TIMER5_BASE + 0x28);
	uint32_t* ARR = (uint32_t*)(TIMER5_BASE + 0x2C);

	//ftimer (div: 1) = 16 000 000 cnt --> 1000 ms
	//		 (div: 16 000 ) = 1    cnt --> 1 ms
	*PSC = 16000 - 1;
	*ARR = 0xffff;

	//timer capture - Channel 1 - rising
	uint16_t* CCER = (uint16_t*)(TIMER5_BASE + 0x20);
	uint16_t* CCMR1 = (uint16_t*)(TIMER5_BASE + 0x18);
	*CCER &= ~( (1<< 1) | (1<<3));		//select rising for TI1FP1
	*CCMR1 |= 0b01;						//select TI1FP1 for IC1
	*CCER |= 1;							//enable capture ch 1
	//timer capture - Channel 2 - falling
	*CCER &= ~( (1<< 7)); *CCER |= ( (1<< 5));	//select falling for TI1FP1
	*CCMR1 |= 0b10 << 8;						//select TI1FP2 for IC2
	*CCER |= 1 << 4;							//enable capture ch 2

	//timer slave mode control - reset cnt when rising
	uint16_t* SMCR = (uint16_t*)(TIMER5_BASE + 0x08);
	*SMCR |= 0b100;		//select RESET mode
	*SMCR |= 0b101 << 4;//select trigger source is TI1FP1

	*CR1 |= 1;			//enable count
}

#define GPIO_BASE_ADD 0x40020400
#define UART_BASE_ADD 0x40011000
uint32_t* SR = (uint32_t*)(UART_BASE_ADD + 0x00);
uint32_t* DR = (uint32_t*)(UART_BASE_ADD + 0x04);
volatile char buffer[32] = {0};
int uart_index = 0;
void UART1_Rx_Handler()
{
	buffer[uart_index] = (char)(*DR);
	*SR &= ~(1>>5);				//clear RXNE flag
	if(uart_index++ >= sizeof(buffer)) uart_index = 0;
}


void UART1_Init()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* MODER = (uint32_t*)(GPIO_BASE_ADD);
	uint32_t* AFRL  = (uint32_t*)(GPIO_BASE_ADD + 0x20);
	*MODER &= ~(0b1111<<12);
	*MODER |= (0b10 << 12) | (0b10 << 14);
	*AFRL |= (0b0111 << 24)| (0b0111 << 28);

	__HAL_RCC_USART1_CLK_ENABLE();
	uint32_t* CR1 = (uint32_t*)(UART_BASE_ADD + 0x0c);
	uint32_t* BRR = (uint32_t*)(UART_BASE_ADD + 0x08);
	//Fuart = 16Mhz
	//Baudrate = 9600
	// 16 000 000 / (16*9600) = 104.166667
	//man = 104
	//fra = 0.1666667*16 = 2.666 ~ 3
	*BRR = (104 << 4) | 3;

//	*CR1 |= (1<< 5);		//enable RXNE interrupt
//	uint32_t* NVIC_ISER1 = (uint32_t*)0xe000e104;
//	*NVIC_ISER1 |= (1 << (37-32)); //enable interrupt in 37 position
//	int* func = 0x200000d4;
//	*func = (int)UART1_Rx_Handler | 1;

	uint32_t* CR3 = (uint32_t*)0x40011014;

	*CR3 |= (1<<6);

	*CR1 |= (1<< 13) | (1<< 3) | (1<< 2);
}
void UART1_Send(char data)
{
	uint32_t* SR = (uint32_t*)(UART_BASE_ADD + 0x00);
	uint32_t* DR = (uint32_t*)(UART_BASE_ADD + 0x04);
	while(((*SR >> 7) & 1) != 1);
	*DR = data;
	while(((*SR >> 6) & 1) != 1);
	*SR &= ~(1<<6);
}

void DMA_Init()
{
	__HAL_RCC_DMA2_CLK_ENABLE();
	uint32_t* S2CR 		= (uint32_t*)0x40026440;	//control
	uint32_t* S2NDTR 	= (uint32_t*)0x40026444;	//number of data
	uint32_t* S2PAR 	= (uint32_t*)0x40026448;	//peripherial address
	uint32_t* S2M0AR 	= (uint32_t*)0x4002644c;	//memory address

	*S2M0AR = (uint32_t)buffer;
	*S2NDTR = sizeof(buffer);
	*S2PAR  = (uint32_t)(0x40011004);  //uart1_dr

	*S2CR |= (0b100 << 25) | (1<<10) | (1<<8) | (1<<0);

}

void SPI_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(0x40020000);
	uint32_t* GPIOA_AFRL  = (uint32_t*)(0x40020020);
	*GPIOA_MODER &= ~(0b111111 << 10);
	*GPIOA_MODER |= (0b10 << 10) | (0b10 << 12) | (0b10 << 14);
	*GPIOA_AFRL &= ~(0xfff << 20);
	*GPIOA_AFRL |= (5<<20) | (5 << 24) | (5<<28);

	__HAL_RCC_GPIOE_CLK_ENABLE();
	uint32_t* GPIOE_MODER = (uint32_t*)(0x40021000);
	*GPIOE_MODER |= (0b01 << 6);
	uint32_t* GPIOE_ODR = (uint32_t*)(0x40021014);
	*GPIOE_ODR |= 1<<3;

	__HAL_RCC_SPI1_CLK_ENABLE();
	uint32_t* CR1 = (uint32_t*)(0x40013000);
	*CR1 |= 1<<1;
	*CR1 |= (0b11<<3);			//set baundrate as Fpclk/16 = 1MHz
	*CR1 |= (1<<8) | (1<<9); 	//enable software management
	*CR1 |= (0b1 <<6) | (1<<2); //enable SPI in master mode
}

void SPI_Write(uint8_t data)
{
	uint8_t temp = 0;
	uint32_t* DR = (uint32_t*)(0x4001300c);
	uint32_t* SR = (uint32_t*)(0x40013008);

	while(((*SR >> 1)&1)!=1);
	*DR = data;
	while(((*SR >> 7)&1)==1);

	while(((*SR >> 0)&1)!=1);	//if Rx buffer has data
	temp = *DR;					//clear Rx buffer
	while(((*SR >> 7)&1)==1);

}

uint8_t SPI_Read()
{
	uint8_t temp = 0;
	uint32_t* DR = (uint32_t*)(0x4001300c);
	uint32_t* SR = (uint32_t*)(0x40013008);

	while(((*SR >> 1)&1)!=1);
	*DR = 0xff;
	while(((*SR >> 7)&1)==1);

	while(((*SR >> 0)&1)!=1);	//if Rx buffer has data
	temp = *DR;					//clear Rx buffer
	while(((*SR >> 7)&1)==1);

	return temp;
}

void LSM303_Active()
{
	uint32_t* GPIOE_ODR = (uint32_t*)(0x40021014);
	*GPIOE_ODR &= ~(1<<3);		//active slave
}

void LSM303_Inactive()
{
	uint32_t* GPIOE_ODR = (uint32_t*)(0x40021014);
	*GPIOE_ODR |= (1<<3);	//in-active slave
}

void LSM303_Init()
{
	LSM303_Active();
	/* enable x, y, z axis
	 * Power mode: 10Hz
	 */
	SPI_Write(0x20);
	SPI_Write(0x27);

	/* data resolution is 12bit (set HR to 1)
	 * Power mode: 10Hz
	 * Enable SPI
	 */
	SPI_Write(0x23 );
	SPI_Write((1<<3) | 1);

	LSM303_Inactive();
}

int16_t LSM303_Read_X()
{
	LSM303_Active();
	SPI_Write(0x28 | (1<<7));
	uint8_t low = SPI_Read();

	SPI_Write(0x29| (1<<7));
	uint8_t high = SPI_Read();
	LSM303_Inactive();

	int16_t result = low | (high << 8);
	if(result > 0x8000)
		result -= (0xffff+1);
	result /= 16;
	return result;
}

uint16_t LSM303_Read_Y()
{
	LSM303_Active();
	SPI_Write(0x2A | (1<<7));
	uint8_t low = SPI_Read();

	SPI_Write(0x2B| (1<<7));
	uint8_t high = SPI_Read();
	LSM303_Inactive();
	int16_t result = low | (high << 8);
	if(result > 0x8000)
		result -= (0xffff+1);
	result /= 16;
	return result;
}
uint16_t LSM303_Read_Z()
{
	LSM303_Active();
	SPI_Write(0x2C | (1<<7));
	uint8_t low = SPI_Read();

	SPI_Write(0x2D| (1<<7));
	uint8_t high = SPI_Read();
	LSM303_Inactive();

	int16_t result = low | (high << 8);

	if(result > 0x8000)
		result -= (0xffff+1);
	result /= 16;

	return result;
}
uint8_t LSM303_Read_ID()
{
	uint8_t WHO_AM_I = 0x0f | (1<<7);	//read ID
	LSM303_Active();
	SPI_Write(WHO_AM_I);
	uint8_t ID = SPI_Read();
	LSM303_Inactive();
	return ID;
}

void I2C1_LSM303_init()
{
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* MODER = (uint32_t*)(0x40020400);
	*MODER |= (0b10 << 12) | (0b10 << 18);

	uint32_t* OTYPER = (uint32_t*)(0x40020404);
	*OTYPER |= (0b1 << 6) | (0b1 << 9);

	//	uint32_t* PUPDR = (uint32_t*)(0x4002040c);
	//	*PUPDR |= (0b01 << 12) | (0b01 << 18);

	uint32_t* AFRL = (uint32_t*)(0x40020420);
	*AFRL &= ~(0b1111 << 24);
	*AFRL |= (4 << 24);

	uint32_t* AFRH = (uint32_t*)(0x40020424);
	*AFRH &= ~(0b1111 << 4);
	*AFRH |= (4 << 4);

	uint32_t* CR1 = (uint32_t*)0x40005400;
	*CR1 &= ~1;

	uint32_t* CR2 = (uint32_t*)0x40005404;
	*CR2 |= 16;

	uint32_t* CCR = (uint32_t*)0x4000541c;
	*CCR |= 320;

	*CR1 |= 1;
}

void I2C1_LSM303_ReadID()
{
	uint32_t* CR1 = (uint32_t*)0x40005400;
	uint32_t* DR  = (uint32_t*)0x40005410;
	uint32_t* SR1  = (uint32_t*)0x40005414;
	uint32_t* SR2  = (uint32_t*)0x40005418;

	const unsigned char SLAVE_ADDR = 0b0011001;

	while(((*SR2 >> 1) &1) == 1);
	*CR1 |= 1<<8;				//generate start bit
	while(((*SR1 >> 0) &1) == 0);

	//send slave addr(0b0011001) + write bit (0)
	*DR = (SLAVE_ADDR << 1) | 0;
	while(((*SR1 >> 1) &1) == 0);
	uint32_t temp = *SR2;

	//send WHO_AM_I (0x0f)
	*DR = 0x0f;
	while(((*SR1 >> 2) &1) == 0);

	//wait ACK
	while(((*SR1 >> 10) &1) == 1);

	//generate start bit
	*CR1 |= 1<<8;
	while(((*SR1 >> 0) &1) == 0);
	//send slave addr(0b0011001) + read bit (1)
	*DR = (SLAVE_ADDR << 1) | 1;
	while(((*SR1 >> 1) &1) == 0);
	temp = *SR2;

	//read data from slave
	uint8_t data = *DR;

	//generate stop bit
	*CR1 |= 1<<9;

}


void I2C_Scan_Slave()
{
	uint32_t* CR1 = (uint32_t*)0x40005400;
	uint32_t* DR  = (uint32_t*)0x40005410;
	uint32_t* SR1  = (uint32_t*)0x40005414;
	uint32_t* SR2  = (uint32_t*)0x40005418;

	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* MODER_D  = (uint32_t*)0x40020c00;
	uint32_t* ODR_D    = (uint32_t*)0x40020c14;
	*MODER_D |= 0b01 << 8;
	*ODR_D &= ~(1<< 4);
	HAL_Delay(5000);
	*ODR_D |= 1<< 4;
	HAL_Delay(1000);

	for(int i = 0; i <= 127; i++)
	{
		while(((*SR2 >> 1) &1) == 1);
		*CR1 |= 1<<8;				//generate start bit
		while(((*SR1 >> 0) &1) == 0);

		*DR = (i<<1) |0;
		char time = 0;
		while(((*SR1 >> 1) &1) == 0)
		{
			if(time++ > 10)
				break;
			HAL_Delay(1);
		}
		if(time < 10)
			__asm("NOP");
		uint32_t temp = *SR2;

		//generate stop bit
		*CR1 |= 1<<9;
		HAL_Delay(1);
	}


}

void I2C1_Audio_ReadID()
{
	uint32_t* CR1 = (uint32_t*)0x40005400;
	uint32_t* DR  = (uint32_t*)0x40005410;
	uint32_t* SR1  = (uint32_t*)0x40005414;
	uint32_t* SR2  = (uint32_t*)0x40005418;

	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* MODER_D  = (uint32_t*)0x40020c00;
	uint32_t* ODR_D    = (uint32_t*)0x40020c14;
	*MODER_D |= 0b01 << 8;
	*ODR_D &= ~(1<< 4);
	HAL_Delay(5000);
	*ODR_D |= 1<< 4;
	HAL_Delay(1000);
	const unsigned char SLAVE_ADDR = 0b1001010;

	while(((*SR2 >> 1) &1) == 1);
	*CR1 |= 1<<8;				//generate start bit
	while(((*SR1 >> 0) &1) == 0);

	//send slave addr(0b0011001) + write bit (0)
	*DR = (SLAVE_ADDR<<1) |0;
	while(((*SR1 >> 1) &1) == 0);
	uint32_t temp = *SR2;

	//send WHO_AM_I ()
	*DR = 0x01;
	while(((*SR1 >> 2) &1) == 0);

	//wait ACK
	while(((*SR1 >> 10) &1) == 1);

	//generate start bit
	*CR1 |= 1<<8;
	while(((*SR1 >> 0) &1) == 0);
	//send slave addr(0b0011001) + read bit (1)
	*DR = (SLAVE_ADDR << 1) | 1;
	while(((*SR1 >> 1) &1) == 0);
	temp = *SR2;

	//read data from slave
	uint8_t data = *DR;

	//generate stop bit
	*CR1 |= 1<<9;

}


void Reset_System()
{
	uint32_t* AIRCR = (uint32_t*)(0xE000ED0C);
	*AIRCR |= (0x5FA << 16) | (1<<2); // SYSRESETREQ
}

/* 1s watchdog */
void IWDG_Init()
{
	uint16_t* KR  = (uint16_t*)(0x40003000);
	uint16_t* PR  = (uint16_t*)(0x40003004);
	uint16_t* RLR = (uint16_t*)(0x40003008);
	*KR  = 0x5555;		//enable access to PR and RLR
	*PR  = 3;
	*RLR = 1000;
	*KR  = 0xCCCC;		//start watchdog
}

void Sleep_Init()
{
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_SLEEP_ENABLE();

	uint32_t* CR = (uint32_t*)(0x40007000);
	*CR |= 0b11;
	uint32_t* SCR = (uint32_t*)(0xe000ed10);
	*SCR |= 1<<2;
	asm("WFI");

}


void ADC_Init()
{
	__HAL_RCC_ADC1_CLK_ENABLE();

	uint32_t *CR2 = (uint32_t*)(0x40012008);
	*CR2 |= 1;						//set ADON
	uint32_t *CCR = (uint32_t*)(0x40012304);
	*CCR |= 1<<23;					//enable temp sensor

	*CR2 |= 1<<20;					//enable external trigger for injected

	uint32_t *JSQR = (uint32_t*)(0x40012038);
	*JSQR |= 16;					//set channel 16(temp sensor) for 1st injected
}

int ADC_Read_Val()
{
	uint32_t *CR2 = (uint32_t*)(0x40012008);
	*CR2 |= 1<<22;

	uint32_t *SR = (uint32_t*)(0x40012000);
	while(((*SR >>2)&1) == 0);
	uint32_t *JDR1 = (uint32_t*)(0x4001203c);
	int result = *JDR1 & 0xfff;

	*SR &= ~(1<<2);

	return result;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	//HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  uint32_t* GPIOD_MODER = (uint32_t*)0x40020c00;
  *GPIOD_MODER |= 0b1010101<<24;		//set output mode for PD12, 13, 14 and 15

  uint32_t* GPIOD_OTYPER= (uint32_t*)0x40020c04;
  *GPIOD_OTYPER &= ~(0b1111<<12);

  uint32_t* GPIOD_ODR= (uint32_t*)0x40020c14;

  /***************** BEGIN SET UP GPIO********************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  uint32_t* GPIOA_MODER = (uint32_t*)0x40020000;
  *GPIOA_MODER &= ~(0b11);		//set PA0 in INPUT mode

  uint32_t* GPIOA_PUPDR = (uint32_t*)0x4002000c;
  *GPIOA_PUPDR &= ~(0b11);		//set no pul up/down for PA0

  uint16_t* GPIOA_IDR = (uint16_t*)0x40020010;
  button_state_t button_state = 0;
  /***************** END OF SETiNG FOR GPIO********************/

  /***************** BEGIN SET UP EXIT********************/
  uint32_t* EXTICR1 = (uint32_t*)0x40013808;
  *EXTICR1 &= ~(0b1111);
  uint32_t* RTSR = (uint32_t*)0x40013c08;
  *RTSR |= 1;
  uint32_t* IMR = (uint32_t*)0x40013c00;
  *IMR |= 1;
  /***************** END OF SETTING EXIT********************/

  /***************** BEGIN SET UP NVIC (Core ARM)********************/
  uint32_t* NVIC_ISER0 = (uint32_t*)0xe000e100;
  *NVIC_ISER0 |= 1<<6;
  *NVIC_ISER0 |= 1<<25;			//enable position 25 ( time 1 & time10 UPDATE )
  /***************** END OF SETTING NVIC (Core ARM)********************/




  memcpy((void*)0x20000000, (void*)0x08000000, 0x198);


  uint32_t* VTOR = (uint32_t*)0xe000ed08;
  *VTOR = 0x20000000;
  /* USER CODE END 2 */

  uint32_t* Function = (uint32_t*)0x20000058;
  *Function = (int)MY_Handler | 1;
  uint32_t* TIM1_HANDLER = (uint32_t*)0x200000A4;
  *TIM1_HANDLER = (int)my_time1_handle | 1;			//thumb

  button_state = global;

  TIM1_Init();
  TIM4_PWM_Init();
  Tim1_Cap();
  Modify_PWM(60);
  time5_init();
  UART1_Init();
  DMA_Init();
  //set dong rong xung 50%
  //set chu ky 1Khz
  const char msg[] = "xin chao\r\n";
  ADC_Init();
//  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int temp = ADC_Read_Val();
	  adc_vin = (float)(temp * 3.0f )/4095.0f; //unit: mV
	  float Temp = ((adc_vin - 0.76) / 2.5) + 25;
	  HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void MY_Handler()
{
	uint32_t* PR = (uint32_t*)0x40013c14;
	*PR |= 1;
}

void EXTI0_IRQHandler()
{

	uint32_t* PR = (uint32_t*)0x40013c14;
	*PR |= 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
