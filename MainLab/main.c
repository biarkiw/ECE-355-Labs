

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

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
//#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
//#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)



//Port and peripheral initialization
void myClock_Init(){
  //enable clock for ports A,B & C
  RCC->AHBENR= 0x000E0000;
  //trace_printf("%x \n",RCC->AHBENR);

  //enable clock for ADC & DAC
  RCC->APB2ENR=RCC_APB2ENR_ADCEN;
  //trace_printf("%x \n",RCC->APB2ENR);
  RCC->APB1ENR=RCC_APB1ENR_DACEN;
  //trace_printf("%x \n",RCC->APB1ENR);
}

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
	// Configure PB as output
	GPIOB->MODER = 0x55555555;
	//ensure no pull up, no pull down
	GPIOB->PUPDR = 0x0;

	//GPIOB->ODR =
}

void myGPIOC_Init(){

  //Configure PC1 as analog input
  GPIOC->MODER = 0x03;

  //

}

void myADC_Init() {

	/*//calibrate ADC
	ADC_CR_ADCAL;
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
*/
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
  trace_printf("Calibration factor of: %u", cal);

  // Configure ADC Channel11 as analog input
  ADC->CHSELR |=
  // Configure the ADC1 in continuous mode, Overrun mode, right data alignment with a resolution equal to 12 bits
  // Select Channel 11 and Convert it with 239.5 Cycles as sampling time
  // Enable the ADC peripheral
  // Wait the ADRDY flag
}


//Interrupt Handlers


//Gobal variables



int main(int argc, char* argv[]){

	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myClock_Init();
	myADC_Init();			/*Initialize ADC*/
	///myGPIOA_Init();		/* Initialize I/O port A */
	//myGPIOB_Init();		/*Initialize I/O port B*/
  myGPIOC_Init();   /*Initialize I/O port C */

	while (1){


	}

	return 0;

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
