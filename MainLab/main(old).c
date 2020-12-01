

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

/*
To Do
1.Enable and configure ADC
2.Enable and configure GPIOC
3.Enable and configure DAC
4.Enable and configure GPIOA
// GPIOC Periph clock enable
// ADC1 Periph clock enable
// Configure ADC Channel11 as analog input
// Configure the ADC1 in continuous mode, Overrun mode, right data alignment with a resolution equal to 12 bits
// Select Channel 11 and Convert it with 239.5 Cycles as sampling time
// Enable the ADC peripheral
// Wait the ADRDY flag
*/
// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

//Gobal variables
int potVal = 0;
int resVal = 0;
unsigned char timTrig = 0;
unsigned int time = 0;
unsigned int period = 0;

//Port and peripheral initialization
void myClock_Init(){

  //enable clock for ports A,B & C
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
  //trace_printf("%x \n",RCC->AHBENR);

  //enable clock for ADC
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
  //trace_printf("%x \n",RCC->APB2ENR);

  //enable clock for DAC & TIM2
  RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM2EN;
  //trace_printf("%x \n",RCC->APB1ENR);

}

void myGPIOA_Init(){

	/* Configure PA1 as input & PA4 as analog for DAC */
	GPIOA->MODER |= GPIO_MODER_MODER4;

	/* Ensure no pull-up/pull-down for PA */
	GPIOA->PUPDR = 0x0;

}

void myGPIOB_Init(){

	// Configure PB as output
	GPIOB->MODER = 0x55555555;
	//ensure no pull up, no pull down
	GPIOB->PUPDR = 0x0;

}

void myGPIOC_Init(){

  //Configure PC1 as analog input
  GPIOC->MODER = 0x03;

}

void myADC_Init() {

  /* calibrate ADC */
  if((ADC1->CR & ADC_CR_ADEN)!= 0){
    //if adc is enabled, disable adc
    ADC1->CR |= ADC_CR_ADDIS;
  }
  while ((ADC1->CR & ADC_CR_ADEN)!=0) {
    /* wait until adc is disabled */
  }
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
  ADC1->CR |= ADC_CR_ADCAL;
  while((ADC1->CR & ADC_CR_ADCAL)!=0){
    /* wait for calibration to finish */
  }
  int cal = (ADC1->DR) & 0x3F;
  trace_printf("Calibration factor of: %u\n", cal);

  /* enable ADC */
  if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)  {
    ADC1->ISR |= ADC_ISR_ADRDY;
  }
  ADC1->CR |= ADC_CR_ADEN;
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)  {
    /* wait for ADC to be enabled */
  }

  // Configure the ADC1 in continuous mode, Overrun mode, right data alignment with a resolution equal to 12 bits
  ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD) & ((~ADC_CFGR1_ALIGN) & (~ADC_CFGR1_RES));

  // Configure ADC Channel11 as analog input
  ADC1->CHSELR |= ADC_CHSELR_CHSEL11;

  // Select Channel 11 and Convert it with 239.5 Cycles as sampling time
  ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
  trace_printf("ADC calibrated and configured, starting continuous conversion \n");

  // start the ADC converting
  ADC1->CR |= ADC_CR_ADSTART;

}

void myDAC_Init(){

  //enable DAC1, make sure all other bits are set to 0
  DAC->CR = DAC_CR_EN1;

}

void myTIM2_Init(){

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = 0x01;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER = 0x1;
}

void myEXTI_Init(){
	/* Map EXTI2 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0]  = SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR=0x2;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR= 0x2;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	//NVIC->IP[2] = 0x0;
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


//Interrupt Handlers

void TIM2_IRQHandler(){
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;

	}
}

void EXTI0_1_IRQHandler(){
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0){
		//
		// 1. If this is the first edge:

		//
		// 2. Clear EXTI2 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		//
		if(timTrig == 0){
			//set flag that timer has been trigger
			timTrig = 1;

			//	- Clear count register (TIM2->CNT).
			TIM2->CNT =0X00000000;

			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;

			//    Else (this is the second edge):
		}else if (timTrig == 1){



			//	- Stop timer (TIM2->CR1).
			TIM2->CR1 ^= TIM_CR1_CEN;
			//	- Read out count register (TIM2->CNT).
			time = TIM2->CNT;
			//	- Calculate signal period and frequency.
			time = SystemCoreClock/time;
			period = (1000000/time);

			//	- Print calculated values to the console.
			trace_printf("Frequency: %u Hz \n",time);
			trace_printf("Period: %u microseconds \n", period);

			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.

			//set flag that timer has been read
			timTrig = 0;
		}
		//
		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		//
		EXTI->PR = 0xFFFF;
	}
}


int main(int argc, char* argv[]){

	trace_printf("System clock: %u Hz\n", SystemCoreClock);

  trace_printf("Starting initializations \n");
  /* Initialize peripheral clocks */
	myClock_Init();
  trace_printf("Clocks initialized \n");
  /* Initialize I/O port A */
	myGPIOA_Init();
  trace_printf("GPIOA initialized \n");
  /* Initialize I/O port B */
  myGPIOB_Init();
  trace_printf("GPIOB initialized \n");
  /* Initialize I/O port C */
  myGPIOC_Init();
  trace_printf("GPIOC initialized \n");
  /* Initialize ADC */
  myADC_Init();
  trace_printf("ADC initialized \n");
  /* Initialize DAC */
  myDAC_Init();
  trace_printf("DAC initialized \n");
  /* Initialize TIM2 */
  myTIM2_Init();
  trace_printf("TIM2 initialized \n");
  /* Initialize EXTI */
  myEXTI_Init();
  trace_printf("EXTI initialized \n");

  unsigned char tal = 0;

	while (1){

    if((ADC1->ISR & ADC_ISR_EOC) != 0 ){
      ADC1->ISR &= ~ADC_ISR_EOC;
      potVal = ADC1->DR & 0xFFFF;
      DAC->DHR12R1 = potVal & 0xFFF;
      tal=1;
    }
    if(tal==1){
      resVal = 5000*((double)potVal/4020);
      trace_printf("pot val : %u\n", resVal);
      tal=0;
    }
	}

	return 0;

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
