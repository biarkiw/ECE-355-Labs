

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

/*
	To Do
1.Enable and configure ADC
2.Enable and configure GPIOC
3.Enable and configure DAC
4.Enable and configure GPIOA

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


//Port and peripheral initialization
void myCOM_Init(){

}

void myGPIOA_Init(){
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	//RCC->AHBENR = 0x0200;

	/* Configure PA2 as input */
	// Relevant register: GPIOA->MODER
	//GPIOA->MODER= 0x0;

	/* Ensure no pull-up/pull-down for PA2 */
	// Relevant register: GPIOA->PUPDR
	//GPIOA->PUPDR = 0x0; //potetnial change to 0x3 if not acting as intetnded
}

void myGPIOB_Init(){

}

void myGPIOC_Init(){

  //Configure PC1 as analog input
  GPIOC->MODER = 0x03;

}

void myADC_Init() {

	//calibrate ADC
	ADC_CR_ADCAL= 1;
	trace_printf("Calibrating ADC \n");

	//Wait for calibration to be completed
	while(ADC_CR_ADCAL != 0){
		trace_printf("calibrating... \n");
	}

  //print calibration factor
  int cal = ADC_DR_DATA;
  trace_printf("Calibration factor of: %u", cal);

  //set clock mode
  ADC_CFGR2_CKMODE_0;

	//Enable ADC
	ADC_CR_ADEN;

  //


}

void myTIM2_Init(){
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR= 0x1;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = 0x01;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	//NVIC->IP[3]=0x0;
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	//NVIC->ISER[0]=0x0;
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER = 0x1;
}

void myEXTI_Init(){
	/* Map EXTI2 line to PA2 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0]  &= ~(SYSCFG_EXTICR1_EXTI2);

	/* EXTI2 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR=0x4;

	/* Unmask interrupts from EXTI2 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR= 0x4;

	/* Assign EXTI2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	//NVIC->IP[2] = 0x0;
	NVIC_SetPriority(EXTI2_3_IRQn, 0);

	/* Enable EXTI2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC->ISER[0]=0x1;
	NVIC_EnableIRQ(EXTI2_3_IRQn);
}

//Interrupt Handlers

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler(){
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM1->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;

	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI2_3_IRQHandler(){
	// Declare/initialize your local variables here...
	unsigned int time = 0;
	unsigned int period = 0;

	/* Check if EXTI2 interrupt pending flag is indeed set */
	/*if ((EXTI->PR & EXTI_PR_PR2) != 0){
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
		// 2. Clear EXTI2 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		//
		EXTI->PR = 0xFFFF;
	}*/
}

//Gobal variables



int main(int argc, char* argv[]){

	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myADC_Init();			/*Initialize ADC*/
	///myGPIOA_Init();		/* Initialize I/O port PA */
	//myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */

	while (1){
		/*if(ADC->ISR )
		ADC->CR*/

	}

	return 0;

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
