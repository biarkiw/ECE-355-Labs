

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"


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

/*registers for current project*/
//#define TIMER (TIM3->CNT)   //counter register
#define IOOUT (GPIOB->ODR)  //output register to send data to lcd
#define IOIN (GPIOB->IDR)   //intput register to get data from lcd

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
#define ALPHA_C_LCD ((uint8_t)0x43  //bit def for C
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
unsigned int time = 0;
unsigned int period = 0;
unsigned char print = 0;
//unsigned int CNT = 0;

//Port and peripheral initialization
void myClock_Init(void);
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void lcdStart(void);

//Interrupt Handlers
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void EXTI0_1_IRQHandler(void);

//Functions
void getADC(void);
void prtVals(void);
void lcdPrint(void);
void handshake(void);

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
  /* Initialize LCD */
  lcdStart();
  trace_printf("LCD initialized \n");


	while (1){

    if((ADC1->ISR & ADC_ISR_EOC) != 0 ){
      getADC();
    }
    if(print>15){
      lcdPrint();
			print = 0;
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
  /*double tmp = potVal;
  resVal = 5000*(tmp/4020);*/
  trace_printf("pot val : %u\n", potVal);

  //	- Calculate signal period and frequency.
  time = SystemCoreClock/time;
  period = (1000000/time);

  //	- Print calculated values to the console.
  trace_printf("Frequency: %u Hz \n",time);
  trace_printf("Period: %u microseconds \n", period);

  resVal = 0;
  time = 0;
  period = 0;
  print = 0;
}

void lcdPrint(){
	unsigned int data = 0;

	IOOUT = LCD_RHOME;
	handshake();
	IOOUT = LCD_WRITE|(ALPHA_O_LCD<<8);
	handshake();
	IOOUT = LCD_WRITE|(ALPHA_H_LCD<<8);
	handshake();
	IOOUT = LCD_WRITE|(ALPHA_M_LCD<<8);
	handshake();
	IOOUT = LCD_WRITE|(ALPHA_S_LCD<<8);
	handshake();
	IOOUT = LCD_WRITE|(SYM_COL_LCD<<8);
	handshake();
	unsigned int valPrt = potVal;
	int cntLoop = 0;
	unsigned int range = 10000000;
	unsigned char hasprint= 0;
	while((valPrt>0) ){
		cntLoop = 0;
		if(valPrt>=range){
			while(valPrt>=range){
				valPrt -= range;
				cntLoop++;
			}
			if(cntLoop==1){
				data = LCD_WRITE|(NUM_1_LCD<<8);
			}else if (cntLoop==2){
				data = LCD_WRITE|(NUM_2_LCD<<8);
			}else if (cntLoop==3){
				data = LCD_WRITE|(NUM_3_LCD<<8);
			}else if (cntLoop==4){
				data = LCD_WRITE|(NUM_4_LCD<<8);
			}else if (cntLoop==5){
				data = LCD_WRITE|(NUM_5_LCD<<8);
			}else if (cntLoop==6){
				data = LCD_WRITE|(NUM_6_LCD<<8);
			}else if (cntLoop==7){
				data = LCD_WRITE|(NUM_7_LCD<<8);
			}else if (cntLoop==8){
				data = LCD_WRITE|(NUM_8_LCD<<8);
			}else if (cntLoop==9){
				data = LCD_WRITE|(NUM_9_LCD<<8);
			}
			range = range/10;
			hasprint = 1;
			IOOUT = data;
			handshake();
		}else if(hasprint==1){
			IOOUT = LCD_WRITE|(NUM_0_LCD<<8);
			range = range/10;
		}else{
			range = range/10;
		}

		}

		IOOUT = LCD_SETDRAM;
		handshake();
		IOOUT = LCD_WRITE|(ALPHA_F_LCD<<8);
		handshake();
		IOOUT = LCD_WRITE|(ALPHA_R_LCD<<8);
		handshake();
		IOOUT = LCD_WRITE|(ALPHA_E_LCD<<8);
		handshake();
		IOOUT = LCD_WRITE|(ALPHA_Q_LCD<<8);
		handshake();
		IOOUT = LCD_WRITE|(SYM_COL_LCD<<8);
		handshake();
		valPrt = SystemCoreClock/time;
		cntLoop = 0;
		range = 10000000;
		hasprint= 0;
		while((valPrt>0) ){
			cntLoop = 0;
			if(valPrt>=range){
				while(valPrt>=range){
					valPrt -= range;
					cntLoop++;
				}
				if(cntLoop==1){
					data = LCD_WRITE|(NUM_1_LCD<<8);
				}else if (cntLoop==2){
					data = LCD_WRITE|(NUM_2_LCD<<8);
				}else if (cntLoop==3){
					data = LCD_WRITE|(NUM_3_LCD<<8);
				}else if (cntLoop==4){
					data = LCD_WRITE|(NUM_4_LCD<<8);
				}else if (cntLoop==5){
					data = LCD_WRITE|(NUM_5_LCD<<8);
				}else if (cntLoop==6){
					data = LCD_WRITE|(NUM_6_LCD<<8);
				}else if (cntLoop==7){
					data = LCD_WRITE|(NUM_7_LCD<<8);
				}else if (cntLoop==8){
					data = LCD_WRITE|(NUM_8_LCD<<8);
				}else if (cntLoop==9){
					data = LCD_WRITE|(NUM_9_LCD<<8);
				}
				range = range/10;
				hasprint = 1;
				IOOUT = data;
				handshake();
			}else if(hasprint==1){
				IOOUT = LCD_WRITE|(NUM_0_LCD<<8);
				range = range/10;
			}else{
				range = range/10;
			}

		}
}


void handshake(){
	while((IOIN & LCD_READBF)==0){

	}
	IOOUT = 0X00000000;
	while((IOIN & LCD_READBF)!=0){

	}

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

      // trigger print
      print++;

			//set flag that timer has been read
			timTrig = 0;
		}
		// Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR = 0xFFFF;
	}
}


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

	// Configure PB in alternet function mode
	GPIOB->MODER = 0x55551555;
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

void lcdStart(){

  IOOUT = LCD_FUNCTSET|LCD_ENABLE;

	handshake();
  IOOUT = LCD_PCONT|LCD_ENABLE;

	handshake();
  IOOUT = LCD_CLEAR|LCD_ENABLE;
	handshake();
  IOOUT = LCD_EMS|LCD_ENABLE;

	handshake();

  IOOUT = LCD_WRITE|(ALPHA_O_LCD<<8);
	handshake();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
