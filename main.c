#include "stm32f401xc.h"
#include "stdint.h"
#include "stdbool.h"

bool monoOperation = true;

//funftion prototypes
void configureSystemClock(void);
void configureGPIO(void);
void configureI2S(void);
void configureDMA(void);

#define BUFFER_LENGTH 48*2

uint16_t audioBuffer[BUFFER_LENGTH] = 
{	
	32767,	32767,
	33836,	33836,
	34887,	34887,
	35902,	35902,
	36863,	36863,
	37754,	37754,
	38560,	38560,
	39266,	39266,
	39861,	39861,
	40335,	40335,
	40680,	40680,
	40889,	40889,
	40959,	40959,
	40889,	40889,
	40680,	40680,
	40335,	40335,
	39861,	39861,
	39266,	39266,
	38560,	38560,
	37754,	37754,
	36863,	36863,
	35902,	35902,
	34887,	34887,
	33836,	33836,
	32767,	32767,
	31698,	31698,
	30647,	30647,
	29632,	29632,
	28671,	28671,
	27780,	27780,
	26974,	26974,
	26268,	26268,
	25673,	25673,
	25199,	25199,
	24854,	24854,
	24645,	24645,
	24575,	24575,
	24645,	24645,
	24854,	24854,
	25199,	25199,
	25673,	25673,
	26268,	26268,
	26974,	26974,
	27780,	27780,
	28671,	28671,
	29632,	29632,
	30647,	30647,
	31698,	31698
};

volatile uint16_t count = 0;

void DMA1_Stream4_IRQHandler(void) {							//DMA1 (I2S) interrupt handler
	if(DMA1->HISR & DMA_HISR_TCIF4)									//handle: DMA transfer complete
	{	
		count++;
		DMA1->HIFCR |= DMA_HIFCR_CTCIF4;							//clear flag: DMA transfer complete
	}
	else if(DMA1->HISR & DMA_HISR_HTIF4)						//handle: DMA half transfer
	{
		//count++;
		DMA1->HIFCR |= DMA_HIFCR_CHTIF4;							//clear flag: DMA half transfer
	}
	
	//============================================== Debug led toggle
	if (count == 1000)
	{
		GPIOC->ODR |= GPIO_ODR_OD13;
	}
	else if (count == 2000)
	{
		GPIOC->ODR &= ~GPIO_ODR_OD13;
		count = 0;
	}
	//============================================== Debug led toggle
}
int main()
{		
	configureSystemClock();
	configureGPIO();
	configureI2S();
	configureDMA();

	while(1)
	{
		
	}
}

void configureSystemClock(void)													//Configure system clock for 84MHz in case of an external 25MHz oscillator
{
	//============================ Configure system clock ====================================
	FLASH->ACR   |= FLASH_ACR_LATENCY; 										//one wait state (fix for PLLRDY flag not enabling)
	//FLASH->ACR |= FLASH_ACR_PRFTEN;  											// prefetch enable
	
	//HSE enable
	RCC->CR 		 |= RCC_CR_HSEON;													//Enable HSE clock
	while(!(RCC->CR & RCC_CR_HSERDY));										//Wait for HSE to lock
	
	//PLL config
	RCC->PLLCFGR &=	~(RCC_PLLCFGR_PLLQ);									//reset values
	RCC->PLLCFGR &=	~(RCC_PLLCFGR_PLLM);
	RCC->PLLCFGR &=	~(RCC_PLLCFGR_PLLN);
	
	RCC->PLLCFGR |=  (7 << RCC_PLLCFGR_PLLQ_Pos);					//PLL48CK output prescaler 									(to  48 MHz)
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
	RCC->CFGR 			&= ~(RCC_CFGR_HPRE);										//no division (AHB = 84MHz)
	RCC->CFGR 			|=   RCC_CFGR_PPRE1_2;									//divide by 2	(APB1 low speed  = 42MHz)
	RCC->CFGR 			&= ~(RCC_CFGR_PPRE2);										//no division (APB2 highs peed = 84MHz)
	RCC->APB1ENR 		|=   RCC_APB1ENR_PWREN;									//enable APB1
	RCC->APB1ENR  	|= 	 RCC_APB1ENR_SPI2EN;								//SPI2 clock enable (for I2S2)
}

void configureGPIO(void)
{	
	/* PC13 = led
	 * 
	 * PB12 = I2S2 WS
	 * PB13 = I2S2 CK
	 * PB15 = I2S2 SD
	 */

	RCC->AHB1ENR   |=   RCC_AHB1ENR_GPIOBEN; 								//enable port B and C		
	RCC->AHB1ENR   |=   RCC_AHB1ENR_GPIOCEN;			
	
	//configure C13 as an output (on board LED)
	GPIOC->MODER   |=   GPIO_MODER_MODE13_0;								//C13 is an output
	GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT13);									//C13 is push pull
	GPIOC->OSPEEDR |= 	GPIO_OSPEEDER_OSPEEDR13;						//C13 max speed
	
	//configure I2S2 IO
	GPIOB->MODER   |=   GPIO_MODER_MODE12_1; 								//I2S2 pins alternate mode
	GPIOB->MODER   |=   GPIO_MODER_MODE13_1; 
	GPIOB->MODER   |=   GPIO_MODER_MODE15_1;
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT12); 									//I2S2 pins push-pull
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT13);
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT15);
	GPIOB->OSPEEDR |=   GPIO_OSPEEDER_OSPEEDR12;						//very high speed
	GPIOB->OSPEEDR |=   GPIO_OSPEEDER_OSPEEDR13;
	GPIOB->OSPEEDR |=   GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD12); 								//I2S2 no pull-up of pull-down
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD13);
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD15);
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL12_0 | GPIO_AFRH_AFSEL12_2);	//I2S2 alternate function select (AF05)
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL13_0 | GPIO_AFRH_AFSEL13_2);
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_2);
}

void configureI2S(void)																		//Configure system clock for 84MHz in case of an external 25MHz oscillator
{	
	//============================ Configure I2S module ====================================
	SPI2->I2SPR 	&= ~(SPI_I2SPR_MCKOE);										//disable master clock output
	SPI2->I2SPR 	&= ~(SPI_I2SPR_I2SDIV);										//reset value to 0
	SPI2->I2SPR 	|= 	(12 << SPI_I2SPR_I2SDIV_Pos); 				//I2S prescaler and odd ( (12*2) + 1 = 25)	(to 1.536 MHz)
	SPI2->I2SPR 	|=   SPI_I2SPR_ODD;
	
	SPI2->I2SCFGR &= ~(SPI_I2SCFGR_CKPOL); 									//clock steady state is low
	SPI2->I2SCFGR &= ~(SPI_I2SCFGR_I2SSTD);									//I2S phillips standard
	SPI2->I2SCFGR &= ~(SPI_I2SCFGR_DATLEN);									//data length is 16 bit
	SPI2->I2SCFGR &= ~(SPI_I2SCFGR_CHLEN);									//channel length is 16 bit
	SPI2->I2SCFGR |= 	 SPI_I2SCFGR_I2SCFG_1;								//master and transmit
	SPI2->I2SCFGR |= 	 SPI_I2SCFGR_I2SMOD;
	SPI2->I2SCFGR |=	 SPI_I2SCFGR_I2SE;										//enable I2S module
	
	//Enable DMA
	SPI2->CR2 		|= 	 SPI_CR2_TXDMAEN;											//enable DMA transmit
}

void configureDMA(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;											//Enable DMA1 clock
	DMA1_Stream4->CR &= ~DMA_SxCR_EN;												//Disable stream		EN SxCR
	while(DMA1_Stream4->CR & DMA_SxCR_EN);									//Wait for stream to stop while(EN)
	
	DMA1->LIFCR = 0x0F7D0F7D;																//Clear LISR and HISR
	DMA1->HIFCR = 0x0F7D0F7D;
	
	DMA1_Stream4->PAR 	=  (uint32_t)(&SPI2->DR);						//set periperal address DMA_SxPAR
	DMA1_Stream4->M0AR 	=  (uint32_t)audioBuffer;						//set memory adresses DMA_SxMA0R and DMA_SxMA1R
	DMA1_Stream4->NDTR 	=   BUFFER_LENGTH;									//total number of data items to be transferred DMA_SxNDTR
	DMA1_Stream4->CR 	 &=  ~DMA_SxCR_CHSEL;									//Select DMA channel  CHSEL[2:0] DMA_SxCR
	DMA1_Stream4->CR 	 |=   DMA_SxCR_PL;										//Configure stream priority PL in DMA_SxCR
			
	DMA1_Stream4->CR 	 |= 	DMA_SxCR_DIR_0;									//direction: memory to periperal 
	DMA1_Stream4->CR   |= 	DMA_SxCR_MINC;									//Memory increment mode
	DMA1_Stream4->CR   &= ~(DMA_SxCR_PINC);									//Peripheral fixed mode
	DMA1_Stream4->CR   &= ~(DMA_SxCR_MBURST | DMA_SxCR_PBURST);	//single transactions, 
	DMA1_Stream4->CR   |= 	DMA_SxCR_MSIZE_0;								//Memory data widths is 16 bit
	DMA1_Stream4->CR   |= 	DMA_SxCR_PSIZE_0;								//Peripheral data widths is 16 bit 
	DMA1_Stream4->CR   |= 	DMA_SxCR_CIRC;									//Circular mode
	DMA1_Stream4->CR   |=  (DMA_SxCR_TCIE | DMA_SxCR_HTIE);	//interrupts after half and/or full in DMA_SxCR
	
	DMA1_Stream4->CR   |=   DMA_SxCR_EN;										//Activate the stream by setting the EN bit in the DMA_SxCR register.	
	
	__NVIC_SetPriority(DMA1_Stream4_IRQn, 2);								//Set I2S global interrupt priority
	__NVIC_EnableIRQ(DMA1_Stream4_IRQn);										//Enable I2S global interrupt
}
