#include <stm32f4xx.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "allbus.h"

#define true 1
#define false 0
	
#define PLL_M 24
#define PLL_N 336
#define PLL_P 2
#define PLL_Q 4

#define mGPIO_AF_FSMC0  0x0000000C
#define mGPIO_AF_FSMC1  0x000000C0
#define mGPIO_AF_FSMC2  0x00000C00
#define mGPIO_AF_FSMC3  0x0000C000
#define mGPIO_AF_FSMC4  0x000C0000
#define mGPIO_AF_FSMC5  0x00C00000
#define mGPIO_AF_FSMC6  0x0C000000
#define mGPIO_AF_FSMC7  0xC0000000
#define mGPIO_AF_FSMC8  0x0000000C
#define mGPIO_AF_FSMC9  0x000000C0
#define mGPIO_AF_FSMC10  0x00000C00
#define mGPIO_AF_FSMC11  0x0000C000
#define mGPIO_AF_FSMC12  0x000C0000
#define mGPIO_AF_FSMC13  0x00C00000
#define mGPIO_AF_FSMC14  0x0C000000
#define mGPIO_AF_FSMC15  0xC0000000
#define ALTERA_BASE      0x60000000

#define MASTER_ALLBUS_ADDR		1
#define MY_ALLBUS_ADDR				6
#define ALLBUS_COM_CRC_REQ 		1
#define ALLBUS_COM_CHANGE_OUT 3

#define OUT_MESSAGE_SIZE_COM_1  6
#define OUT_MESSAGE_SIZE_COM_3  24

#define U3_BUFFER_SIZE (1 << 8)
#define CntAdd(cnt, add, size) ((cnt + add) & (size - 1))
#define CntInc(cnt, size) ((cnt + 1) & (size - 1))

#define DIODE_SWITCH 500
#define DIODE_HL1 1
#define DIODE_HL2 (1 << 1)
#define DIODE_HL3 (1 << 2)

uint16_t U3RX_cnt, U3TX_cnt, U3TX_size, U3RX_cnt_slave; 
uint8_t U3RX_buf[U3_BUFFER_SIZE], U3TX_buf[U3_BUFFER_SIZE], Out_Data[U3_BUFFER_SIZE];


uint8_t *I2C_data = Out_Data + 26-5;
uint8_t I2C_addr[3] = {0x20 << 1, 0x21 << 1, 0x22 << 1};
int fl_i2c_start;
int i2c_cnt;
int diode_sensor_cnt = 0, diode_msg_cnt = 0;
int diode_sensor_fl = 0, diode_msg_fl = 0;
int rs422_start_en = true;
uint16_t altera_diodes = 0;


short angles[4];

typedef struct
{
	uint32_t r0:		2;
	uint32_t sq25:	2;
	uint32_t sq26:	2;
	uint32_t r1:		8;
	uint32_t sq31:	2;
	uint32_t sq32:	2;
	uint32_t r2:		10;
	uint32_t sq38:	2;
	uint32_t sq39:	2;
	uint32_t sq40:	2;
	uint32_t sq41:	2;
	uint32_t r3:		12;
} SQs;

SQs altera_sq;

void mPLL_init(void)
{
	int StartUpCounter = 0;
	
	RCC->CR |= RCC_CR_HSEON;
	while(((RCC->CR & RCC_CR_HSERDY) == 0) || (++StartUpCounter < 1000)){;} 
		
/* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

		    /* HCLK = SYSCLK / 1*/
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | 
								RCC_CFGR_PPRE2_DIV2 | 
								RCC_CFGR_PPRE1_DIV4;
		
	RCC->PLLCFGR = 	RCC_PLLCFGR_PLLSRC_HSE	|//HSE oscillator clock selected as PLL
									(PLL_Q << 24) |//PLLQ
									(((PLL_P >> 1) -1) << 16) |//PLLP
									(PLL_N << 6) |//PLLN
									PLL_M;//PLLM
		
	RCC->CR |= RCC_CR_PLLON;
		
	while((RCC->CR & RCC_CR_PLLRDY) == 0){;} 	
		
    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
		
    /* Select the main PLL as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= RCC_CFGR_SW_PLL;	
		
	while((RCC->CFGR & RCC_CFGR_SWS_1) == 0){;} 	
}

void EXTI_init()
{
	//PC0
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER &= ~(GPIO_MODER_MODER0);//input
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1;
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	
	EXTI->RTSR = EXTI_FTSR_TR0;
	EXTI->FTSR = 0;
	EXTI->IMR = EXTI_IMR_MR0;
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PC ;
	
	NVIC_EnableIRQ(EXTI0_IRQn);
	*(short*)(ALTERA_BASE + (0x9 << 1))=
			0 | 			//altera selfstart
			(1 << 1); //int0 enable
}

void mI2C_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR11_1;
	GPIOB->AFR[1] |= (4 << 8) | (4 << 12);

	NVIC_EnableIRQ(I2C2_EV_IRQn);
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	I2C2->CR1 = I2C_CR1_SWRST;//reset
	I2C2->CR1 = 0;
	I2C2->CCR = 207;
	I2C2->CR2 = I2C_CR2_ITEVTEN;

	I2C2->CR1 |= I2C_CR1_PE; 
	fl_i2c_start = false;
	i2c_cnt = -1;
}
void mUart_init()
{
	/*
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN;
	//RS-232
	GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11;
	GPIOC->AFR[1] |= (7 << 8) | (7 << 12);

	NVIC_EnableIRQ(USART3_IRQn);
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
//	USART3->BRR = (45 << 4)| 8;//= 42MHz / (8 * (2 - 0) * 45.562)115.2
//	USART3->BRR = (22 << 4)| 12;//= 42MHz / (8 * (2 - 0) * 22.786)230.4
	USART3->BRR = (11 << 4)| 3;//= 42MHz / (8 * (2 - 0) * 11.393)460.8	
	USART3->CR3 = 1 << 7;//DMA mode is enabled for transmission
	USART3->CR1 = 
			USART_CR1_OVER8 |
			USART_CR1_UE |
			USART_CR1_TE |
			USART_CR1_RE |
			USART_CR1_RXNEIE;
	
*/		
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//RS-422
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER0_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3| GPIO_OSPEEDER_OSPEEDR0;
	GPIOA->AFR[0] |= (7 << 8) | (7 << 12);

	NVIC_EnableIRQ(USART2_IRQn);
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR = (11 << 4)| 3;//= 42MHz / (8 * (2 - 0) * 11.393)460.8	
	
	USART2->CR3 = 1 << 7;//DMA mode is enabled for transmission
	USART2->CR1 = 
			USART_CR1_OVER8 |
			USART_CR1_UE |
			USART_CR1_TE |
			USART_CR1_RE |
			USART_CR1_RXNEIE;	
}

void mDMA_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	DMA1_Stream6->CR = 
			(4 << 25) | //Channel 4
			(1 << 10) | //Memory address pointer is incremented after each data transfer
			(1 << 6); //Memory-to-peripheral
	DMA1_Stream6->PAR = (uint32_t)&(USART2->DR);
	DMA1_Stream6->M0AR = (uint32_t)&U3TX_buf;
}

void mFSMC_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN 
							| RCC_AHB1ENR_GPIOEEN 
							| RCC_AHB1ENR_GPIOFEN 
							| RCC_AHB1ENR_GPIOGEN;
	
	GPIOD->MODER = GPIO_MODER_MODER0_1	
							|	GPIO_MODER_MODER1_1
							|	GPIO_MODER_MODER3_1	
							|	GPIO_MODER_MODER4_1	
							|	GPIO_MODER_MODER5_1
							|	GPIO_MODER_MODER7_1	
							|	GPIO_MODER_MODER8_1	
							|	GPIO_MODER_MODER9_1	
							|	GPIO_MODER_MODER10_1
							|	GPIO_MODER_MODER14_1
							|	GPIO_MODER_MODER15_1;
							
	GPIOD->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0	
							|	GPIO_OSPEEDER_OSPEEDR1
							|	GPIO_OSPEEDER_OSPEEDR3	
							|	GPIO_OSPEEDER_OSPEEDR4	
							|	GPIO_OSPEEDER_OSPEEDR5
							|	GPIO_OSPEEDER_OSPEEDR7	
							|	GPIO_OSPEEDER_OSPEEDR8	
							|	GPIO_OSPEEDER_OSPEEDR9	
							|	GPIO_OSPEEDER_OSPEEDR10
							|	GPIO_OSPEEDER_OSPEEDR14	
							|	GPIO_OSPEEDER_OSPEEDR15;
								
	GPIOD->AFR[0] = mGPIO_AF_FSMC0 
							| mGPIO_AF_FSMC1 
							| mGPIO_AF_FSMC3
							| mGPIO_AF_FSMC4
							| mGPIO_AF_FSMC5
							| mGPIO_AF_FSMC7;
	GPIOD->AFR[1] = mGPIO_AF_FSMC8 
							| mGPIO_AF_FSMC9 
							| mGPIO_AF_FSMC10
							| mGPIO_AF_FSMC14
							| mGPIO_AF_FSMC15;

	GPIOE->MODER = GPIO_MODER_MODER0_1	
							|	GPIO_MODER_MODER1_1
							|	GPIO_MODER_MODER7_1	
							|	GPIO_MODER_MODER8_1	
							|	GPIO_MODER_MODER9_1
							|	GPIO_MODER_MODER10_1	
							|	GPIO_MODER_MODER11_1	
							|	GPIO_MODER_MODER12_1	
							|	GPIO_MODER_MODER13_1
							|	GPIO_MODER_MODER14_1
							|	GPIO_MODER_MODER15_1;
							
	GPIOE->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0	
							|	GPIO_OSPEEDER_OSPEEDR1
							|	GPIO_OSPEEDER_OSPEEDR7	
							|	GPIO_OSPEEDER_OSPEEDR8	
							|	GPIO_OSPEEDER_OSPEEDR9
							|	GPIO_OSPEEDER_OSPEEDR10	
							|	GPIO_OSPEEDER_OSPEEDR11	
							|	GPIO_OSPEEDER_OSPEEDR12	
							|	GPIO_OSPEEDER_OSPEEDR13
							|	GPIO_OSPEEDER_OSPEEDR14	
							|	GPIO_OSPEEDER_OSPEEDR15;
								
	GPIOE->AFR[0] = mGPIO_AF_FSMC0 
							| mGPIO_AF_FSMC1 
							| mGPIO_AF_FSMC7;
	GPIOE->AFR[1] = mGPIO_AF_FSMC8 
							| mGPIO_AF_FSMC9 
							| mGPIO_AF_FSMC10 
							| mGPIO_AF_FSMC11
							| mGPIO_AF_FSMC12
							| mGPIO_AF_FSMC13
							| mGPIO_AF_FSMC14
							| mGPIO_AF_FSMC15;

	GPIOF->MODER = GPIO_MODER_MODER0_1	
							|	GPIO_MODER_MODER1_1
							|	GPIO_MODER_MODER2_1	
							|	GPIO_MODER_MODER3_1	
							|	GPIO_MODER_MODER4_1
							|	GPIO_MODER_MODER5_1	
							|	GPIO_MODER_MODER12_1	
							|	GPIO_MODER_MODER13_1
							|	GPIO_MODER_MODER14_1
							|	GPIO_MODER_MODER15_1;
							
	GPIOF->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0	
							|	GPIO_OSPEEDER_OSPEEDR1
							|	GPIO_OSPEEDER_OSPEEDR2	
							|	GPIO_OSPEEDER_OSPEEDR3	
							|	GPIO_OSPEEDER_OSPEEDR4
							|	GPIO_OSPEEDER_OSPEEDR5	
							|	GPIO_OSPEEDER_OSPEEDR12	
							|	GPIO_OSPEEDER_OSPEEDR13
							|	GPIO_OSPEEDER_OSPEEDR14	
							|	GPIO_OSPEEDER_OSPEEDR15;
								
	GPIOF->AFR[0] = mGPIO_AF_FSMC0 
							| mGPIO_AF_FSMC1 
							| mGPIO_AF_FSMC2 
							| mGPIO_AF_FSMC3 
							| mGPIO_AF_FSMC4
							| mGPIO_AF_FSMC5;
	GPIOF->AFR[1] = mGPIO_AF_FSMC12
							| mGPIO_AF_FSMC13
							| mGPIO_AF_FSMC14
							| mGPIO_AF_FSMC15;

	GPIOG->MODER = GPIO_MODER_MODER0_1	
							|	GPIO_MODER_MODER1_1
							|	GPIO_MODER_MODER2_1	
							|	GPIO_MODER_MODER3_1	
							|	GPIO_MODER_MODER4_1;
							
	GPIOG->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0	
							|	GPIO_OSPEEDER_OSPEEDR1
							|	GPIO_OSPEEDER_OSPEEDR2	
							|	GPIO_OSPEEDER_OSPEEDR3	
							|	GPIO_OSPEEDER_OSPEEDR4;
								
	GPIOG->AFR[0] = mGPIO_AF_FSMC0 
							| mGPIO_AF_FSMC1 
							| mGPIO_AF_FSMC2 
							| mGPIO_AF_FSMC3 
							| mGPIO_AF_FSMC4;

	RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;
	FSMC_Bank1->BTCR[0] = 0x00003011;//0x00003011;
	FSMC_Bank1->BTCR[1] = 0x021408F2;//0x021408F2;
	FSMC_Bank1E->BWTR[0] = 0x021F08F2;//0x021F08F2;
}

void EXTI0_IRQHandler()//mmbk ang ready
{
	EXTI->PR |= EXTI_PR_PR0;
	angles[0] = *(short*)(ALTERA_BASE + (0xA << 1));//angle 1
	angles[1] = *(short*)(ALTERA_BASE + (0xB << 1));//angle 2
	angles[2] = *(short*)(ALTERA_BASE + (0xC << 1));//angle 3
	angles[3] = *(short*)(ALTERA_BASE + (0xD << 1));//angle 4
	diode_sensor_cnt++;
}

void I2C2_EV_IRQHandler()
{
	int sr1 = I2C2->SR1;
	if(sr1 & I2C_SR1_SB) I2C2->DR = I2C_addr[i2c_cnt];
	if(sr1 & I2C_SR1_ADDR) {if(I2C2->SR2){;}}
	if(((sr1 & (I2C_SR1_TXE | I2C_SR1_BTF)) == (I2C_SR1_TXE | I2C_SR1_BTF)) && ((I2C2->CR1 & I2C_CR1_STOP) == 0))
	{				
		I2C2->CR1 |= I2C_CR1_STOP;	
		fl_i2c_start = true;
	}
	else if(sr1 & I2C_SR1_TXE) I2C2->DR = I2C_data[i2c_cnt];
}

void USART2_IRQHandler()
{	
	if(USART2->SR & USART_SR_RXNE)
	{
		U3RX_buf[U3RX_cnt] = USART2->DR;
		U3RX_cnt = CntInc(U3RX_cnt, U3_BUFFER_SIZE);
	}
	USART2->SR = 0;
}

void form_message_mmbk()
{
	uint16_t data;
	altera_sq = *(SQs*)(ALTERA_BASE + (3 << 1));

	data = *(uint16_t*)(ALTERA_BASE + (8 << 1));
	Out_Data[5-5] = (uint8_t)data;
	Out_Data[6-5] = (uint8_t)(data >> 8);
	
	Out_Data[11-5] = (uint8_t)(altera_sq.sq41 << 4 | altera_sq.sq32 << 6);
	Out_Data[12-5] = (uint8_t)(altera_sq.sq26 | (altera_sq.sq40 << 2) | (altera_sq.sq31 << 4) | (altera_sq.sq25 << 6));
	Out_Data[13-5] = (uint8_t)(altera_sq.sq39 | (altera_sq.sq38 << 2));

	data = angles[0];//angle 1
	Out_Data[14-5] = (uint8_t)(data >> 8);
	Out_Data[15-5] = (uint8_t)data;
	data = angles[1];//angle 2
	Out_Data[16-5] = (uint8_t)(data >> 8);
	Out_Data[17-5] = (uint8_t)data;
	data = angles[2];//angle 3
	Out_Data[18-5] = (uint8_t)(data >> 8);
	Out_Data[19-5] = (uint8_t)data;
	data = angles[3];//angle 4
	Out_Data[20-5] = (uint8_t)(data >> 8);
	Out_Data[21-5] = (uint8_t)data;
}

void applicate_in_messages(uint8_t command, uint8_t first)
{
	uint32_t message_size; 
	uint32_t fl_send_answer = false;
	switch(command)
	{
		case ALLBUS_COM_CRC_REQ: 
			Out_Data[0] = 0x05;
			Out_Data[1] = 0x10;
			Out_Data[2] = 0x03;
			Out_Data[3] = 0x15;
			Out_Data[4] = 0x03;
			Out_Data[5] = 0x15;
			message_size = OUT_MESSAGE_SIZE_COM_1;
			fl_send_answer = true;
		break;
		case ALLBUS_COM_CHANGE_OUT:
			I2C_data[0] = ~U3RX_buf[first++];
			I2C_data[1] = ~U3RX_buf[first++];
			I2C_data[2] = ~U3RX_buf[first++];
   		i2c_cnt = 3;
			fl_i2c_start = true;
			form_message_mmbk();
			message_size = OUT_MESSAGE_SIZE_COM_3;
			fl_send_answer = true;
		break;
	}
	if(fl_send_answer)
	{
		if(rs422_start_en){
			rs422_start_en = false;
			GPIOA->ODR |= GPIO_ODR_ODR_0; //rs422 enable
		}	
	  ALLBUS_send(MASTER_ALLBUS_ADDR, command, message_size, U3TX_buf, Out_Data);
		USART2->SR &= ~USART2->SR & USART_SR_TC;
		DMA1->HIFCR |= DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6;
		DMA1_Stream6->NDTR = message_size + 7;
		DMA1_Stream6->CR |= 1;
		diode_msg_cnt++;
	}
}
void income_messages()
{
	uint8_t com, addr;
	int U3RX_rez;
	while(Compare_counters(U3RX_cnt_slave, U3RX_cnt, 5, U3_BUFFER_SIZE))
	{
		U3RX_rez = ALLBUS_recieve(U3RX_buf, U3RX_cnt_slave, U3RX_cnt, (int)&com, (int)&addr);
		if(U3RX_rez >= 0)//message had gotten; 
		{
			if(addr == MY_ALLBUS_ADDR) applicate_in_messages(com, CntAdd(U3RX_cnt_slave, 5, U3_BUFFER_SIZE));
			U3RX_cnt_slave = U3RX_rez;
			break;
		}
		if(U3RX_rez == -1) break;//message not full
		if(U3RX_rez == -2) U3RX_cnt_slave = CntInc(U3RX_cnt_slave, U3_BUFFER_SIZE);//header error, follow to next element
	}
}

int main()
{	
	volatile int i = 0;
	while(i++ < 0x100000);
	
	mPLL_init();
	mFSMC_init();
	mUart_init();
	mDMA_init();
	mI2C_init();
	EXTI_init();

	altera_diodes = DIODE_HL3;
	
	while(1)
	{	
		income_messages();
		if(fl_i2c_start && ((I2C2->CR1 & I2C_CR1_STOP) == 0)) 
		{
			fl_i2c_start = false;
			if(--i2c_cnt >= 0)	I2C2->CR1 |= I2C_CR1_START;
		}
		if(diode_msg_cnt >= DIODE_SWITCH) 
		{
			diode_msg_cnt = 0;
			if(diode_msg_fl) altera_diodes |= DIODE_HL1;
			else altera_diodes &= ~DIODE_HL1;
			diode_msg_fl = !diode_msg_fl;
		}
		if(diode_sensor_cnt >= DIODE_SWITCH) 
		{		
			diode_sensor_cnt = 0;
			if(diode_sensor_fl) altera_diodes |= DIODE_HL2;		
			else altera_diodes &= ~DIODE_HL2;
			diode_sensor_fl = !diode_sensor_fl;
		}
		*(uint32_t*)(ALTERA_BASE + (15 << 1)) = altera_diodes;

	}	
	return 0;
}


