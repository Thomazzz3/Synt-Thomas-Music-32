#include "stm32f401xc.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

//=========================================================== interpolate from arduino synth
#define SAMPLE_RATE 48000
struct sampledWave
{
  uint8_t waveSize;
  int16_t wave[32];
  float frequency;
  float increasePerSample;
  uint8_t currentSample;
  float remainder;
};

struct sampledWave sine =
{
  32,
  {
		0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 
		16068, 15136, 13622, 11585, 9102, 6269, 3196, 0, -3196, 
		-6269, -9102, -11585, -13622, -15136, -16068, -16383, 
		-16068, -15136, -13622, -11585, -9102, -6269, -3196
  },
  880.0f,
  0.0f,
  0,
  0.0f
};

struct sampledWave square =
{
  32,
  {
		-8000, -7000, -6000, -5000, -4000, -3000, -2000, -1000,     
		    0,  1000,  2000,  3000,  4000,  5000,  6000,  7000,
		 8000,  7000,  6000,  5000,  4000,  3000,  2000,  1000,     
		    0, -1000, -2000, -3000, -4000, -5000, -6000, -7000,		
  },
  440.0f,
  0.0f,
  0,
  0.0f
};

struct sampledWave square2 =
{
  32,
  {
		-8000, -7000, -6000, -5000, -4000, -3000, -2000, -1000,     
		    0,  1000,  2000,  3000,  4000,  5000,  6000,  7000,
		 8000,  7000,  6000,  5000,  4000,  3000,  2000,  1000,     
		    0, -1000, -2000, -3000, -4000, -5000, -6000, -7000,		
  },
  440.0f,
  0.0f,
  0,
  0.0f
};

volatile bool handlePingBuffer = false;
volatile bool handlePongBuffer = false;

//funftion prototypes
void configureSystemClock(void);
void configureGPIO(void);
void configureI2S(void);
void configureDMA(void);
void configureADC(void);
int16_t interpolation(struct sampledWave *wave);
void calcIncreasePerSample(struct sampledWave *wave);

//audio buffer
#define BUFFER_LENGTH 48*2
uint16_t audioBuffer[BUFFER_LENGTH];

void DMA1_Stream4_IRQHandler(void)								//DMA1 (I2S) interrupt handler
{							
	if(DMA1->HISR & DMA_HISR_TCIF4)									//handle: DMA transfer complete
	{	
		handlePongBuffer = true;
		DMA1->HIFCR |= DMA_HIFCR_CTCIF4;							//clear flag: DMA transfer complete
	}
	else if(DMA1->HISR & DMA_HISR_HTIF4)						//handle: DMA half transfer
	{
		handlePingBuffer = true;
		DMA1->HIFCR |= DMA_HIFCR_CHTIF4;							//clear flag: DMA half transfer
	}
	
	asm("nop");//delete
	asm("nop");
	asm("nop");
}

uint16_t adcReading = 0;

void ADC_IRQHandler(void)													//ADC interrupt handler
{							
	if(ADC1->SR & ADC_SR_EOC)												//handle: ADC end of conversion
	{
		adcReading =	ADC1->DR;
		sine.frequency = (float)(adcReading/5);
		calcIncreasePerSample(&sine);

		ADC1->SR &= ~ADC_SR_EOC;											//clear flag: DMA transfer complete
	}
	
	asm("nop");//delete
	asm("nop");
	asm("nop");
}

int16_t calculateSample()
{
	int16_t output;
	
	//================= cool synth sutff here ===========================
	
	output = interpolation(&sine)/5;
	
	//output = interpolation(&square);
	//output += interpolation(&square2)/2;
	
	//================= all done for this sample ========================
	
	return output;
}

uint16_t counter = 0;

void handlePingPongBuffer()
{
	int16_t currentSampleOut;
	
	if(handlePingBuffer)
	{
		for(uint8_t i = 0; i < (BUFFER_LENGTH/4); i++)
		{
			currentSampleOut = calculateSample();
			audioBuffer[ i*2     ] = currentSampleOut;
			audioBuffer[(i*2) + 1] = currentSampleOut;
		}
		
		counter++;
		
		handlePingBuffer = false;
	}
	if(handlePongBuffer)
	{
		for(uint8_t i = 0; i < (BUFFER_LENGTH/4); i++)
		{
			currentSampleOut = calculateSample();
			audioBuffer[(BUFFER_LENGTH/2) + (i*2)    ] = currentSampleOut;
			audioBuffer[(BUFFER_LENGTH/2) + (i*2) + 1] = currentSampleOut;
		}
		handlePongBuffer = false;
	}
}

int main()
{		
	configureSystemClock();
	configureGPIO();
	configureI2S();
	configureDMA();
	configureADC();
	
	calcIncreasePerSample(&sine);
	calcIncreasePerSample(&square);
	while(1)
	{
		//this is where the audio buffer is filled
		handlePingPongBuffer();
		
		/*
		if(counter == 179)
		{
			square.frequency = 164.81f;
			calcIncreasePerSample(&square);
		}
		else if(counter == 357)
		{
			square.frequency = 196;
			calcIncreasePerSample(&square);
		}
		else if(counter == 535)
		{
			square.frequency = 246.94f;
			calcIncreasePerSample(&square);
		}
		else if(counter == 714)
		{
			square.frequency = 261.63f;
			calcIncreasePerSample(&square);
		}
		else if(counter == 892)
		{
			square.frequency = 246.94f;
			calcIncreasePerSample(&square);
		}
		else if(counter == 1071)
		{
			square.frequency = 196;
			calcIncreasePerSample(&square);
		}
		else if(counter == 1250)
		{
			square.frequency = 164.81f;
			calcIncreasePerSample(&square);
		}
		else if(counter >= 1428)
		{
			square.frequency = 130.81f;
			calcIncreasePerSample(&square);
			counter = 0;
		}
		*/
	}
}

void configureSystemClock(void)													//Configure system clock for 84MHz in case of an external 25MHz oscillator
{
	//============================ Configure system clock ====================================
	FLASH->ACR   |= FLASH_ACR_LATENCY; 										//one wait state (fix for PLLRDY flag not enabling)
	
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
	//				I2S bitrate = 16 bit * 2 channels * 48kHz = 1,536 MHz

	 
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
	//============================ Configure GPIO ====================================
	/* PC13 = led
	 * 
	 * PB12 = I2S2 WS
	 * PB13 = I2S2 CK
	 * PB15 = I2S2 SD
	 *
	 * PA1  = ADC1
	 * PA2  = ADC2
	 * PA3  = ADC3
	 * PA4  = ADC4
	 */
	
	RCC->AHB1ENR   |=   RCC_AHB1ENR_GPIOAEN; 								//enable port A, B and C	
	RCC->AHB1ENR   |=   RCC_AHB1ENR_GPIOBEN;	
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
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD12); 								//I2S2 no pull-up or pull-down
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD13);
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD15);
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL12_0 | GPIO_AFRH_AFSEL12_2);	//I2S2 alternate function select (AF05)
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL13_0 | GPIO_AFRH_AFSEL13_2);
	GPIOB->AFR[1]  |=  (GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_2);
	
	//configure ADC1 IO
	GPIOA->MODER	 |=		GPIO_MODER_MODE1;										//analog mode
	GPIOA->MODER	 |=		GPIO_MODER_MODE2;
	GPIOA->MODER	 |=		GPIO_MODER_MODE3;
	GPIOA->MODER	 |=		GPIO_MODER_MODE4;
	GPIOA->OSPEEDR |=		GPIO_OSPEEDER_OSPEEDR1;							//very high speed
	GPIOA->OSPEEDR |=		GPIO_OSPEEDER_OSPEEDR2;
	GPIOA->OSPEEDR |=		GPIO_OSPEEDER_OSPEEDR3;
	GPIOA->OSPEEDR |=		GPIO_OSPEEDER_OSPEEDR4;
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD1); 									//ADC1 no pull-up or pull-down
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD2);
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD3);
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD4);
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
	//============================ Configure DMA for I2S ====================================
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

void configureADC(void)
{
	//============================ Configure ADC ====================================
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		//ADC1 clock enable
	
	ADC1->CR1 &= ~ADC_CR1_RES;			//12 bit resolution
	ADC1->CR1 |= 	ADC_CR1_SCAN;			//enable scan mode
	ADC1->CR1 |= 	ADC_CR1_EOCIE;		//end of conversion interrupt enabled
	ADC1->CR2 &= ~ADC_CR2_ALIGN;		//data right aligned
	ADC1->CR2 |=	ADC_CR2_EOCS;			//EOC is set after each conversion
	ADC1->CR2 |=	ADC_CR2_CONT;			//continuous conversion
	
	ADC1->SMPR2 |= (ADC_SMPR2_SMP1 |		//sampling time is 12 + 480 ADC clock cycles
									ADC_SMPR2_SMP2 |		//channels 1 to 4
									ADC_SMPR2_SMP3 |								
									ADC_SMPR2_SMP4);	
	
	//ADC1->SQR1 |=		ADC_SQR1_L_1;					//4 conversions
	ADC1->SQR3 |=	((ADC_SQR3_SQ1 & 1) |		//the channel scan order is: 1, 2, 3, 4
								 (ADC_SQR3_SQ2 & 2) |
								 (ADC_SQR3_SQ3 & 3) |
								 (ADC_SQR3_SQ4 & 4));
	
	ADC->CCR |=			ADC_CCR_ADCPRE;			//ADC prescaler is 8  (84MHZ/8 = 10.5MHz) (max 36MHz)
	
	
	ADC1->CR2 |= 	ADC_CR2_ADON;			//enable ADC
	ADC1->CR2 |= ADC_CR2_SWSTART;		//start conversion of regular channels 
	
	
	
	__NVIC_SetPriority(ADC_IRQn, 4);		//Set ADC global interrupt priority
	__NVIC_EnableIRQ(ADC_IRQn);					//Enable ADC global interrupt	
}

void calcIncreasePerSample(struct sampledWave *wave)
{
  wave->increasePerSample = (wave->frequency * wave->waveSize) / SAMPLE_RATE;
}


int16_t interpolation(struct sampledWave *wave)
{
  //update current sample
  float newSampleNr = wave->increasePerSample + wave->remainder;
  
  //increase current sample (and ignore remainder)
  wave->currentSample += (uint8_t)newSampleNr;

  //if current sample falls out of the wave size, correct it
  while(wave->currentSample >= wave->waveSize)
  {
    wave->currentSample -= wave->waveSize;
  }

  //update remainder
  wave->remainder = fmodf(newSampleNr, 1);

  //================================ interpolate
  float slope;
  
	//normal interpolation
  if(wave->currentSample != wave->waveSize - 1)
  {
    slope = (wave->wave[wave->currentSample] - wave->wave[wave->currentSample+1]) / 32768.0f;
  }
	//current sample is the last one in the wave. interpolate between last and first sample 
  else
  {
    slope = (wave->wave[wave->currentSample] - wave->wave[0]) / 32768.0f;
  }

  return wave->wave[wave->currentSample] - ((slope * wave->remainder) * 32768);
}

