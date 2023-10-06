/*
File Created by: Biarki Weeks & Fraser Jones
With reference to the ECE 355 Lab Manual and Lecture Notes
in addition to various online resources and tutorials
*/

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0-hal\stm32f0xx_hal_spi.h"
#include "stm32f0-hal/stm32f0xx_hal_spi_ex.h"

// ----------------------------------------------------------------------------

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
#define myTIM3_PRESCALER ((uint16_t) 0xFFFF)  // LCD refresh rate
#define myTIM6_PRESCALER ((uint16_t) 4)
SPI_HandleTypeDef hspi1;




/*general opperation bit definition for current project*/
#define LCD_CLEAR ((uint16_t)0x0110)    //clear display
#define LCD_RHOME ((uint16_t)0x0210)    //return home
#define LCD_EMS ((uint16_t)0x0600)      //entry mode set
#define LCD_PCONT ((uint16_t)0x0C00)    //display ON/OFF control
#define LCD_CURSOR ((uint16_t)0x1000)   //cursor/display shift
#define LCD_FUNCTSET ((uint16_t)0x3800) //function set (0X3C00) to change font
#define LCD_BUSY ((uint16_t)0x8000)     //busy flag from lcd
#define LCD_READBF ((uint16_t)0x0080)   //set to read busy flag
#define LCD_LINE1 ((uint8_t)0x00)       //DDRAM address for line 1
#define LCD_LINE2 ((uint8_t)0x40)       //DDRAM address for line 2
#define LCD_SETDRAM ((uint16_t)0xC010)     //set DDRAM address
#define LCD_WRITE ((uint16_t)0x0030)     //write do lcd, must be or'd with data
#define LCD_ENABLE ((uint16_t)0x0010)

/*character bit definition*/
//capital letters
#define ALPHA_A_LCD ((uint8_t)0x41) //bit def for A
#define ALPHA_B_LCD ((uint8_t)0x42) //bit def for B
#define ALPHA_C_LCD ((uint8_t)0x43)  //bit def for C
#define ALPHA_D_LCD ((uint8_t)0x44) //bit def for D
#define ALPHA_E_LCD ((uint8_t)0x45) //bit def for E
#define ALPHA_F_LCD ((uint8_t)0x46) //bit def for F
#define ALPHA_G_LCD ((uint8_t)0x47) //bit def for G
#define ALPHA_H_LCD ((uint8_t)0x48) //bit def for H
#define ALPHA_I_LCD ((uint8_t)0x49) //bit def for I
#define ALPHA_J_LCD ((uint8_t)0x4A) //bit def for J
#define ALPHA_K_LCD ((uint8_t)0x4B) //bit def for K
#define ALPHA_L_LCD ((uint8_t)0x4C) //bit def for L
#define ALPHA_M_LCD ((uint8_t)0x4D) //bit def for M
#define ALPHA_N_LCD ((uint8_t)0x4E) //bit def for N
#define ALPHA_O_LCD ((uint8_t)0x4F) //bit def for O
#define ALPHA_P_LCD ((uint8_t)0x50) //bit def for P
#define ALPHA_Q_LCD ((uint8_t)0x51) //bit def for Q
#define ALPHA_R_LCD ((uint8_t)0x52) //bit def for R
#define ALPHA_S_LCD ((uint8_t)0x53) //bit def for S
#define ALPHA_T_LCD ((uint8_t)0x54) //bit def for T
#define ALPHA_U_LCD ((uint8_t)0x55) //bit def for U
#define ALPHA_V_LCD ((uint8_t)0x56) //bit def for V
#define ALPHA_W_LCD ((uint8_t)0x57) //bit def for W
#define ALPHA_X_LCD ((uint8_t)0x58) //bit def for X
#define ALPHA_Y_LCD ((uint8_t)0x59) //bit def for Y
#define ALPHA_Z_LCD ((uint8_t)0x5A) //bit def for Z
//symbols
#define SYM_COL_LCD ((uint8_t)0x3A) // bit definition for :
//numbers
#define NUM_0_LCD ((uint8_t)0x30)  //bit def for 0
#define NUM_1_LCD ((uint8_t)0x31)  //bit def for 1
#define NUM_2_LCD ((uint8_t)0x32)  //bit def for 2
#define NUM_3_LCD ((uint8_t)0x33)  //bit def for 3
#define NUM_4_LCD ((uint8_t)0x34)  //bit def for 4
#define NUM_5_LCD ((uint8_t)0x35)  //bit def for 5
#define NUM_6_LCD ((uint8_t)0x36)  //bit def for 6
#define NUM_7_LCD ((uint8_t)0x37)  //bit def for 7
#define NUM_8_LCD ((uint8_t)0x38)  //bit def for 8
#define NUM_9_LCD ((uint8_t)0x39)  //bit def for 9
//Gobal variables
unsigned int potVal = 0;
double resVal = 0;
unsigned char timTrig = 0;
float freq = 0;
unsigned char print = 0;
//unsigned int CNT = 0;

//Port and peripheral initialization
void myClock_Init(void);
void SystemClock48MHz(void);
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void mySPI_Init(void);
void lcdStart(void);
void myTIM3_Init(void);
void myTIM6_Init(void);

//Interrupt Handlers
void TIM2_IRQHandler(void);
void EXTI0_1_IRQHandler(void);

//Functions
void getADC(void);
void prtVals(void);
void wait(int microseconds);
void writeToLCD(uint8_t, int);   // 4-bit interface (high and low nibble) for the final call of HC595
void HC595Write(uint8_t);  // SPI: LCK and MOSI running

// ----- main() ---------------------------------------------------------------
int main(int argc, char* argv[]){
	
	
	trace_printf("Starting initializations \n");
	/* Initialize peripheral clocks */
		myClock_Init();
	trace_printf("Clocks initialized \n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);
	/* Initialize I/O port A */
		myGPIOA_Init();
	trace_printf("GPIOA initialized \n");
	/* Initialize I/O port B */
	myGPIOB_Init();
	GPIOB -> ODR |= GPIO_PIN_4;
	trace_printf("GPIOB initialized \n");
	/* Initialize ADC */
	myADC_Init();
	trace_printf("ADC initialized \n");
	/* Initialize DAC */
	myDAC_Init();
	trace_printf("DAC initialized \n");
	/* Initialize SPI*/
		mySPI_Init();
	/* Initialize TIM2 */
	myTIM2_Init();
	myTIM3_Init();
	myTIM6_Init();
	trace_printf("TIM2, TIM3 & TIM6 initialized \n");
	/* Initialize EXTI */
	myEXTI_Init();
	trace_printf("EXTI initialized \n");

	/* Initialize LCD */
	myLCD_Init();
	trace_printf("LCD initialized \n");
	uint8_t data = 0x85;
	GPIOB -> ODR=0x0000;
		while (1){

		if((ADC1->ISR & ADC_ISR_EOC) != 0 ){
		getADC();
		}
	
		}

		return 0;
}

//Functions
void getADC(){
  ADC1->ISR &= ~ADC_ISR_EOC;
  potVal = ADC1->DR & 0xFFFF;
  DAC->DHR12R1 = potVal & 0xFFF;
}

void prtVals(){
	/*used to print to the console*/
	trace_printf("pot val : %u\n", potVal);

	//	- Calculate signal period and frequency.

	unsigned int frequency = (unsigned int)freq;
	//	- Print calculated values to the console.
	trace_printf("Frequency: %d Hz \n",frequency);

	resVal = 0;

}

void wait(int microseconds){
	// 12 cycles with prescaler = 1 microsecond
	TIM6->ARR = 12 * microseconds;

	// Set count to 0
	TIM6->CNT = 0;
	// Start timer
	TIM6->CR1 |= 0x1;

	// Clear status register
	TIM6->SR = 0;
	// Wait until timer overflows
	while((TIM6->SR & 0x1) == 0);

	// Stop timer
	TIM6->CR1 &= 0xFFFE;

}

void writeToLCD(uint8_t c, int isCommand){
	/* Function not tested*/
    uint8_t RS = isCommand ? 0x00 : 0x40;
    uint8_t EN = 0x80;

    uint8_t highNibble = (c & 0xF0) >> 4;
    uint8_t lowNibble = c & 0xF;

    HC595Write(highNibble | LCD_WRITE);
    HC595Write(highNibble | LCD_WRITE | LCD_ENABLE);
    HC595Write(highNibble | LCD_WRITE);
    HC595Write(lowNibble | LCD_WRITE);
    HC595Write(lowNibble | LCD_WRITE | LCD_ENABLE);
    HC595Write(lowNibble | LCD_WRITE);
}

void HC595Write(uint8_t data){

		/* Force the LCK signal to 0 */
	GPIOB -> ODR &= ~GPIO_PIN_4;

  //wait(2);

		/* Assumption: your data holds 8 bits to be sent */
	HAL_SPI_Transmit(&hspi1, &data, 1, 1);



		/* Force the LCK signal to 1 */
	GPIOB -> ODR |= GPIO_PIN_4;

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
void TIM3_IRQHandler(){
	/*Used to send inturupt to regularily print to console, intended for use with screen however proper writing to the screen was not achieved*/

	/* Check if update interrupt flag is indeed set */
	if ((TIM3->SR & TIM_SR_UIF) != 0)	{
    prtVals();

	/* Clear update interrupt flag */
	// Relevant register: TIM2->SR
	TIM3->SR &= 0xFFFE;

	// Should correspond to a value of 1 second
	TIM3->CNT = SystemCoreClock / myTIM3_PRESCALER;

	/* Restart stopped timer */
	// Relevant register: TIM2->CR1
	TIM3->CR1 |= 0x1;
	}
}

void EXTI0_1_IRQHandler(){
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0){

		if(timTrig == 0){
		
			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;

      	//set flag that timer has been trigger
			timTrig = 1;

			//    Else (this is the second edge):
		}else if (timTrig == 1){

			

			//	- Read out count register (TIM2->CNT).
			int time = TIM2->CNT;

      //	- Stop timer (TIM2->CR1).
			TIM2->CR1 ^= TIM_CR1_CEN;

	    //	- Clear count register (TIM2->CNT).
			TIM2->CNT =0X00000000;

      double signalPeriod = time / (double) SystemCoreClock;
      //unsigned int sig = (unsigned int)signalPeriod;
		//trace_printf("the signalPeriod is , %u \n", sig);

		  freq = (float) (1 / signalPeriod);
      //unsigned int frequency = (unsigned int)freq;
      //trace_printf("the FREQ is , %d \n", frequency);
      
			//set flag that timer has been read
			timTrig = 0;
		}
		// Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR = 0xFFFF;
	}
}

//Port and peripheral initialization
void myClock_Init(){

  void SystemClock48MHz();

  //enable clock for ports A,B & C
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
  //trace_printf("%x \n",RCC->AHBENR);

  //enable clock for ADC
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN | RCC_APB2ENR_SPI1EN;
  //trace_printf("%x \n",RCC->APB2ENR);

  //enable clock for DAC & TIM2
  RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM2EN;
  //trace_printf("%x \n",RCC->APB1ENR);

}

void SystemClock48MHz( void ){
	//
	// Disable the PLL
	//
		RCC->CR &= ~(RCC_CR_PLLON);
	//
	// Wait for the PLL to unlock
	//
		while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );
	//
	// Configure the PLL for a 48MHz system clock
	//
		RCC->CFGR = 0x00280000;

	//
	// Enable the PLL
	//
		RCC->CR |= RCC_CR_PLLON;

	//
	// Wait for the PLL to lock
	//
		while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );

	//
	// Switch the processor to the PLL clock source
	//
		RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

	//
	// Update the system with the new clock frequency
	//
		SystemCoreClockUpdate();

}

void myGPIOA_Init(){

	/* Configure PA1 as input & PA4 as analog for DAC */
	GPIOA->MODER |= 0x0FF0;

	/* Ensure no pull-up/pull-down for PA */
	GPIOA->PUPDR = 0x0;

}

void myGPIOB_Init(){

	/* 
  Configure pin: 
  PB5 as alternate for MOSI
  PB4 as output GPIO for the latch 
  PB3 as alternate for Shifting Clock SCK
  */
	GPIOB->MODER =0x00000980;
}

void myADC_Init(){

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
  ADC1->CHSELR |= ADC_CHSELR_CHSEL5;

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
	TIM2->EGR = 0x01;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER = 0x1;
}

void myTIM3_Init(void){
		/* Enable clock for TIM3 peripheral */
		// Relevant register: RCC->APB1ENR
		RCC->APB1ENR |= RCC_APB1RSTR_TIM3RST;

		/* Configure TIM3: buffer auto-reload, count down, stop on overflow
		/* Enable update events, interrupt on underflow only */
		// Relevant register: TIM3->CR1
		TIM3->CR1 &= 0xFFFD;
		TIM3->CR1 |= 0x9D;

		/* Set clock prescaler value */
		// Frequency is 732Hz
		TIM3->PSC = myTIM3_PRESCALER;

		/* Set auto-reloaded delay */
		TIM3->ARR = myTIM3_PRESCALER;

		// Should correspond to a value of 1 second
		TIM3->CNT = SystemCoreClock / myTIM3_PRESCALER;

		
		/* Update timer registers */
		// Relevant register: TIM3->EGR
		TIM3->EGR |= 0x1;
		/* Assign TIM3 interrupt priority = 0 in NVIC */
		// Relevant register: NVIC->IP[3] or use NVIC_SetPriority
		NVIC_SetPriority(TIM3_IRQn, 0);

		/* Enable TIM3 interrupts in NVIC */
		// Relevant register: NVIC->ISER[0] or use NVIC_EnableIRQ
		NVIC_EnableIRQ(TIM3_IRQn);

		/* Enable update interrupt generation */
		// Relevant register: TIM3->DIER
		TIM3->DIER |= TIM_DIER_UIE;

		// Start timer
		TIM3->CR1 |= 0x1;
}

void myEXTI_Init(){
	/* Map EXTI2 line to PA1 */
	SYSCFG->EXTICR[0]  = SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	EXTI->RTSR=0x2;

	/* Unmask interrupts from EXTI1 line */
	EXTI->IMR= 0x2;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void mySPI_Init(){
	/*Hall SPI Initalization for Master, 1-line, Motorola, 8-bit mode*/

	 hspi1.Instance = SPI1;
	 hspi1.Init.Mode = SPI_MODE_MASTER;
	 hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	 hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	 hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	 hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	 hspi1.Init.NSS = SPI_NSS_SOFT;
	 hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	 hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	 hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  	hspi1.Init.CRCPolynomial = 7;
  	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

	 HAL_SPI_Init(&hspi1);
   __HAL_SPI_ENABLE(&hspi1);

}

void myLCD_Init(){
	/*While the SPI data bus is working, this function is not*/
	// Sends instructions for initializing the LCD unit
	wait(1520);
	HC595Write(0x03);
	wait(1520);
	HC595Write(0x03);
	wait(1520);
	HC595Write(0x03);
	wait(37);
	HC595Write(0x02);
	wait(37);
	HC595Write(0x02);
	wait(37);
	HC595Write(0x08);//N=1, F=0
	wait(37);
	HC595Write(0x00);
	wait(37);
	HC595Write(0x08);
	wait(37);
	HC595Write(0x00);
	wait(37);
	HC595Write(0x01);
	wait(37);
	HC595Write(0x00);
	wait(37);
	HC595Write(0x06); //I/D=1, S=0
	
}


void myTIM6_Init(){
	/* Enable clock for TIM6 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1RSTR_TIM6RST;

	/* Configure TIM6: buffer auto-reload */
	/* Enable update events */
	TIM6->CR1 |= 0x84;

	/* Set clock prescaler value */
	// Frequency is 12MHz
	TIM6->PSC = myTIM6_PRESCALER;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
