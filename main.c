#include "stm32f401xc.h"
#include "stdint.h"

//funftion prototypes
void configureSystemClock(void);
void configureGPIO(void);
void configureI2S(void);

#define BUFFER_LENGTH 8
uint16_t bufferIndex = 0;

uint16_t audioBuffer[BUFFER_LENGTH] = 						//6kHz triangle wave
{
	32767, 32769, 32771, 32769, 32767, 32765, 32763, 32765
};



void SPI2_IRQHandler(void) {
	
	if(SPI2->SR & SPI_SR_FRE)
	{
		GPIOC->ODR ^= GPIO_ODR_OD13;
	}
	else if (SPI2->SR & SPI_SR_TXE)
	{
		if (!(SPI2->SR & SPI_SR_CHSIDE))			//left channel will be sent					
		{
			if (bufferIndex >= BUFFER_LENGTH)		//prevent buffer overflow
			{
				bufferIndex = 0;
			}
			else
			{
				bufferIndex++;
			}
		}	
		SPI2->DR = audioBuffer[bufferIndex];
	}
}

int main()
{		
	configureSystemClock();
	configureGPIO();
	configureI2S();

	while(1)
	{
		//volatile long i = 0;
		//for(i=0;i<0x400000;i++);
		//GPIOC->ODR ^= GPIO_ODR_OD13;
		/*
		if(SPI2->SR & SPI_SR_TXE)								//if write complete
		{
			
			if (!(SPI2->SR & SPI_SR_CHSIDE))			//left channel will be sent					
			{
				if (bufferIndex >= BUFFER_LENGTH)		//prevent buffer overflow
				{
					bufferIndex = 0;
				}
				else
				{
					bufferIndex++;
				}
			}
			
			//GPIOC->ODR ^= GPIO_ODR_OD13;					//debug led toggle
			SPI2->DR = audioBuffer1[bufferIndex];	//write new value
		}	*/
	}
}

void configureSystemClock(void)													//Configure system clock for 84MHz in case of an external 25MHz oscillator
{
	//============================ Configure system clock ====================================
	FLASH->ACR   |= FLASH_ACR_LATENCY; 											//one wait state (fix for PLLRDY flag not enabling)
	//FLASH->ACR |= FLASH_ACR_PRFTEN;  											// prefetch enable
	
	//HSE enable
	RCC->CR 		 |= RCC_CR_HSEON;													//Enable HSE clock
	while(!(RCC->CR & RCC_CR_HSERDY));										//Wait for HSE to lock
	
	//PLL config
	RCC->PLLCFGR &=	~(RCC_PLLCFGR_PLLQ);									//reset values
	RCC->PLLCFGR &=	~(RCC_PLLCFGR_PLLM);
	RCC->PLLCFGR &=	~(RCC_PLLCFGR_PLLN);
	
	RCC->PLLCFGR |=  (7 << RCC_PLLCFGR_PLLQ_Pos);					//PLL48CK output prescaler 									(to   48 MHz)
	RCC->PLLCFGR |=  (25 << RCC_PLLCFGR_PLLM_Pos);				//PLL and PLLI2S input prescaler 						(to   1 MHz)
	RCC->PLLCFGR |=  (336 << RCC_PLLCFGR_PLLN_Pos);				//PLL multiplication factor									(to 336 MHz)
	RCC->PLLCFGR |=   RCC_PLLCFGR_PLLP_0;									//PLL output prescaler to system clock (=4)	(to  84 MHz)
	RCC->PLLCFGR |=   RCC_PLLCFGR_PLLSRC;									//PLL and PLLI2S source is HSE
	RCC->CR      |=   RCC_CR_PLLON;												//Enable PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));										//Wait for PLL to lock 

	//system clock select
	RCC->CFGR 	 |= (RCC_CFGR_SW & 0b10);									//PLL to system clock
	while((RCC->CFGR & RCC_CFGR_SWS) != (0b10 << RCC_CFGR_SWS_Pos)); //Wait for MUX to switch
	
	//============================ Configure I2S clock ====================================
	/* I2S bitrate = 16 bit * 2 channels * 48kHz = 1,536 MHz
	 *
	 */
	 
	//PLLI2S config
	RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SN);						//Reset value to 0
	RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SR);
	
	RCC->PLLI2SCFGR |=  (192 << RCC_PLLI2SCFGR_PLLI2SN_Pos);//PLL I2S multiplication factor							(to 192   MHz)
	RCC->PLLI2SCFGR |=  (5 << RCC_PLLI2SCFGR_PLLI2SR_Pos);	//PLL I2S output prescaler to system clock  (to  38.4 MHz)
	RCC->CFGR 			&= ~(RCC_CFGR_I2SSRC);									//I2S source is PLL I2S
	RCC->CR 				|=   RCC_CR_PLLI2SON;										//Enable PLL I2S
	while(!(RCC->CR & RCC_CR_PLLI2SRDY));										//Wait for PLL I2S to lock
	
	//APB1 config
	RCC->CFGR 		&= ~(RCC_CFGR_HPRE);											//no division (AHB = 84MHz)
	RCC->CFGR 		|=   RCC_CFGR_PPRE1_2;										//divide by 2	(APB1 low speed  = 42MHz)
	RCC->CFGR 		&= ~(RCC_CFGR_PPRE2);											//no division (APB2 highs peed = 84MHz)
	RCC->APB1ENR 	|=   RCC_APB1ENR_PWREN;										//enable APB1
	RCC->APB1ENR  |= 	 RCC_APB1ENR_SPI2EN;									//SPI2 clock enable (for I2S2)
	
	//====================================== extra stuff usefull ???
	//
	//
}

void configureGPIO(void)
{	
	/* PC13 = led
	 * 
	 * PB12 = I2S2 WS
	 * PB13 = I2S2 CK
	 * PB15 = I2S2 SD
	 */

	RCC->AHB1ENR   |=   RCC_AHB1ENR_GPIOBEN; 					//enable port B and C		
	RCC->AHB1ENR   |=   RCC_AHB1ENR_GPIOCEN;			
	
	//configure C13 as an output (on board LED)
	GPIOC->MODER   |=   GPIO_MODER_MODE13_0;					//C13 is an output
	GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT13);							//C13 is push pull
	GPIOC->OSPEEDR |= 	GPIO_OSPEEDER_OSPEEDR13;			//C13 max speed
	
	//configure I2S2 IO
	GPIOB->MODER   |=   GPIO_MODER_MODE12_1; 		//I2S2 pins alternate mode
	GPIOB->MODER   |=   GPIO_MODER_MODE13_1; 
	GPIOB->MODER   |=   GPIO_MODER_MODE15_1;
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT12); 						//I2S2 pins push-pull
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT13);
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT15);
	GPIOB->OSPEEDR |=   GPIO_OSPEEDER_OSPEEDR12;			//very high speed
	GPIOB->OSPEEDR |=   GPIO_OSPEEDER_OSPEEDR13;
	GPIOB->OSPEEDR |=   GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD12); 					//I2S2 no pull-up of pull-down
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD13);
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD15);
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL12_0 | GPIO_AFRH_AFSEL12_2);	//I2S2 alternate function select (AF05)
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL13_0 | GPIO_AFRH_AFSEL13_2);
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_2);
}

void configureI2S(void)															//Configure system clock for 84MHz in case of an external 25MHz oscillator
{	
	//============================ Configure I2S module ====================================
	SPI2->I2SPR 	&= ~(SPI_I2SPR_MCKOE);							//disable master clock output
	SPI2->I2SPR 	&= ~(SPI_I2SPR_I2SDIV);							//reset value to 0
	SPI2->I2SPR 	|= 	(12 << SPI_I2SPR_I2SDIV_Pos); 	//I2S prescaler and odd ( (12*2) + 1 = 25)	(to 1.536 MHz)
	SPI2->I2SPR 	|=   SPI_I2SPR_ODD;
	
	SPI2->I2SCFGR &= ~(SPI_I2SCFGR_CKPOL); 						//clock steady state is low
	SPI2->I2SCFGR &= ~(SPI_I2SCFGR_I2SSTD);						//I2S phillips standard
	SPI2->I2SCFGR &= ~(SPI_I2SCFGR_DATLEN);						//data length is 16 bit
	SPI2->I2SCFGR &= ~(SPI_I2SCFGR_CHLEN);						//channel length is 16 bit
	SPI2->I2SCFGR |= 	 SPI_I2SCFGR_I2SCFG_1;					//master and transmit
	SPI2->I2SCFGR |= 	 SPI_I2SCFGR_I2SMOD;
	SPI2->I2SCFGR |=	 SPI_I2SCFGR_I2SE;							//enable I2S module
	
	//Enable interrupts
	SPI2->CR2 		|= SPI_CR2_TXEIE;										//Enable transmit interrupt
	//SPI2->CR2 	|= SPI_CR2_RXNEIE;										//Enable receive interrupt
	SPI2->CR2 		|= SPI_CR2_ERRIE;										//Enable error interrupt
	__NVIC_SetPriority(SPI2_IRQn, 2);									//Set I2S global interrupt priority
	__NVIC_EnableIRQ(SPI2_IRQn);											//Enable I2S global interrupt
	
}
