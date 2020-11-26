

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
//#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
//#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)



//Port and peripheral initialization
void myClock_Init(){

  //enable clock for ports A,B & C
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
  //trace_printf("%x \n",RCC->AHBENR);

  //enable clock for ADC & DAC
  RCC->APB2ENR=RCC_APB2ENR_ADCEN;
  //trace_printf("%x \n",RCC->APB2ENR);
  RCC->APB1ENR=RCC_APB1ENR_DACEN;
  //trace_printf("%x \n",RCC->APB1ENR);

}

void myGPIOA_Init(){

	/* Configure PA1 as input & PA4 as analog for DAC */
	GPIOA->MODER= 0x00000300;

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

//Interrupt Handlers


//Gobal variables
int potVal = 0;
int resVal = 0;

int main(int argc, char* argv[]){

	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myClock_Init();    /* Initialize peripheral clocks */
	myADC_Init();      /* Initialize ADC */
	myGPIOA_Init();    /* Initialize I/O port A */
	myGPIOB_Init();    /* Initialize I/O port B */
  myGPIOC_Init();    /* Initialize I/O port C */

	while (1){

    if((ADC1->ISR & ADC_ISR_EOC) != 0 ){
      ADC1->ISR &= ~ADC_ISR_EOC;
      potVal = ADC1->DR & 0xFFFF;
      resVal = 5000*(potVal/4020);
      // 5kohm = 4020ish
      trace_printf("pot val : %u\n", resVal);
    }
	}

	return 0;

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
