/*
 * hardlcd.h
 *
 *  Created on: 19 Nov 2020
 *      Author: biarki
 *  this header file is designed for the setup in the ECE 355 lab,
 *  and will be subject to change in the future
 */

#ifndef HARDLCD_H_
#define HARDLCD_H_

#include "cmsis/cmsis_device.h"

/*registers for current project*/
#define TIMER (TIM2->CNT)   //counter register
#define IOOUT (GPIOB->ODR)  //output register to send data to lcd
#define IOIN (GPIOB->IDR)   //intput register to get data from lcd


/*general opperation bit definition for current project*/

#define LCD_CLEAR ((uint16_t)0x0100)    //clear display
#define LCD_RHOME ((uint16_t)0x0200)    //return home
#define LCD_EMS ((uint16_t)0x0600)      //entry mode set
#define LCD_PCONT ((uint16_t)0x0C00)    //display ON/OFF control
#define LCD_CURSOR ((uint16_t)0x1000)   //cursor/display shift
#define LCD_FUNCTSET ((uint16_t)0x3800) //function set (0X3C00) to change font
#define LCD_BUSY ((uint16_t)0x8000)     //busy flag from lcd
#define LCD_READBF ((uint16_t)0x0040)   //set to read busy flag
#define LCD_LINE1 ((uint8_t)0x00)       //DDRAM address for line 1
#define LCD_LINE2 ((uint8_t)0x40)       //DDRAM address for line 2
#define LCD_SETDRAM ((uint8_t)0x80)     //set DDRAM address
#define LCD_WRITE ((uint16_t)0x0030)     //write do lcd, must be or'd with data



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

/*global variables for following functions*/
int CNT = 0;
/*functions*/
void lcdPrint(char line1[16], char line2[16]){
  IOOUT = (LCD_SETDRAM|LCD_LINE1)<<8;
  unsigned int data = 0;
  for(int i=0; (line1[i] != "\0")||(i<16); i++){

    switch (line1[i]) {
      case 'A':
      data = (LCD_WRITE|(ALPHA_A_LCD<<8));
      case 'B':
      data = (LCD_WRITE|(ALPHA_B_LCD<<8));
      case 'C':
      data = LCD_WRITE|(ALPHA_C_LCD<<8);
      case 'D':
      data = LCD_WRITE|(ALPHA_D_LCD<<8);
      case 'E':
      data = LCD_WRITE|(ALPHA_E_LCD<<8);
      case 'F':
      data = LCD_WRITE|(ALPHA_F_LCD<<8);
      case 'G':
      data = LCD_WRITE|(ALPHA_G_LCD<<8);
      case 'H':
      data = LCD_WRITE|(ALPHA_H_LCD<<8);
      case 'I':
      data = LCD_WRITE|(ALPHA_I_LCD<<8);
      case 'J':
      data = LCD_WRITE|(ALPHA_J_LCD<<8);
      case 'K':
      data = LCD_WRITE|(ALPHA_K_LCD<<8);
      case 'L':
      data = LCD_WRITE|(ALPHA_L_LCD<<8);
      case 'M':
      data = LCD_WRITE|(ALPHA_M_LCD<<8);
      case 'N':
      data = LCD_WRITE|(ALPHA_N_LCD<<8);
      case 'O':
      data = LCD_WRITE|(ALPHA_O_LCD<<8);
      case 'P':
      data = LCD_WRITE|(ALPHA_P_LCD<<8);
      case 'Q':
      data = LCD_WRITE|(ALPHA_Q_LCD<<8);
      case 'R':
      data = LCD_WRITE|(ALPHA_R_LCD<<8);
      case 'S':
      data = LCD_WRITE|(ALPHA_S_LCD<<8);
      case 'T':
      data = LCD_WRITE|(ALPHA_T_LCD<<8);
      case 'U':
      data = LCD_WRITE|(ALPHA_U_LCD<<8);
      case 'V':
      data = LCD_WRITE|(ALPHA_V_LCD<<8);
      case 'W':
      data = LCD_WRITE|(ALPHA_W_LCD<<8);
      case 'X':
      data = LCD_WRITE|(ALPHA_X_LCD<<8);
      case 'Y':
      data = LCD_WRITE|(ALPHA_Y_LCD<<8);
      case 'Z':
      data = LCD_WRITE|(ALPHA_Z_LCD<<8);
      case '0':
      data = LCD_WRITE|(NUM_0_LCD<<8);
      case '1':
      data = LCD_WRITE|(NUM_1_LCD<<8);
      case '2':
      data = LCD_WRITE|(NUM_2_LCD<<8);
      case '3':
      data = LCD_WRITE|(NUM_3_LCD<<8);
      case '4':
      data = LCD_WRITE|(NUM_4_LCD<<8);
      case '5':
      data = LCD_WRITE|(NUM_5_LCD<<8);
      case '6':
      data = LCD_WRITE|(NUM_6_LCD<<8);
      case '7':
      data = LCD_WRITE|(NUM_7_LCD<<8);
      case '8':
      data = LCD_WRITE|(NUM_8_LCD<<8);
      case '9':
      data = LCD_WRITE|(NUM_9_LCD<<8);
      default:
      trace_printf("something went wrong!");
    }

    IOOUT = data;
    CNT = TIMER;
    while((0 < (TIMER-CNT))||((TIMER-CNT) < (SystemCoreClock/20000))){
      //delay for 50 microseconds
    }
    IOOUT = LCD_READBF;
    while ((IOIN & LCD_BUSY) != 0) {
      //wait until busy signal is active
    }
  }

  IOOUT = (LCD_SETDRAM|LCD_LINE2)<<8;
  for(int i=0; (line2[i] != "\0")||(i<16); i++){

    switch (line2[i]) {
      case 'A':
      data = LCD_WRITE|(ALPHA_A_LCD<<8);
      case 'B':
      data = LCD_WRITE|(ALPHA_B_LCD<<8);
      case 'C':
      data = LCD_WRITE|(ALPHA_C_LCD<<8);
      case 'D':
      data = LCD_WRITE|(ALPHA_D_LCD<<8);
      case 'E':
      data = LCD_WRITE|(ALPHA_E_LCD<<8);
      case 'F':
      data = LCD_WRITE|(ALPHA_F_LCD<<8);
      case 'G':
      data = LCD_WRITE|(ALPHA_G_LCD<<8);
      case 'H':
      data = LCD_WRITE|(ALPHA_H_LCD<<8);
      case 'I':
      data = LCD_WRITE|(ALPHA_I_LCD<<8);
      case 'J':
      data = LCD_WRITE|(ALPHA_J_LCD<<8);
      case 'K':
      data = LCD_WRITE|(ALPHA_K_LCD<<8);
      case 'L':
      data = LCD_WRITE|(ALPHA_L_LCD<<8);
      case 'M':
      data = LCD_WRITE|(ALPHA_M_LCD<<8);
      case 'N':
      data = LCD_WRITE|(ALPHA_N_LCD<<8);
      case 'O':
      data = LCD_WRITE|(ALPHA_O_LCD<<8);
      case 'P':
      data = LCD_WRITE|(ALPHA_P_LCD<<8);
      case 'Q':
      data = LCD_WRITE|(ALPHA_Q_LCD<<8);
      case 'R':
      data = LCD_WRITE|(ALPHA_R_LCD<<8);
      case 'S':
      data = LCD_WRITE|(ALPHA_S_LCD<<8);
      case 'T':
      data = LCD_WRITE|(ALPHA_T_LCD<<8);
      case 'U':
      data = LCD_WRITE|(ALPHA_U_LCD<<8);
      case 'V':
      data = LCD_WRITE|(ALPHA_V_LCD<<8);
      case 'W':
      data = LCD_WRITE|(ALPHA_W_LCD<<8);
      case 'X':
      data = LCD_WRITE|(ALPHA_X_LCD<<8);
      case 'Y':
      data = LCD_WRITE|(ALPHA_Y_LCD<<8);
      case 'Z':
      data = LCD_WRITE|(ALPHA_Z_LCD<<8);
      case '0':
      data = LCD_WRITE|(NUM_0_LCD<<8);
      case '1':
      data = LCD_WRITE|(NUM_1_LCD<<8);
      case '2':
      data = LCD_WRITE|(NUM_2_LCD<<8);
      case '3':
      data = LCD_WRITE|(NUM_3_LCD<<8);
      case '4':
      data = LCD_WRITE|(NUM_4_LCD<<8);
      case '5':
      data = LCD_WRITE|(NUM_5_LCD<<8);
      case '6':
      data = LCD_WRITE|(NUM_6_LCD<<8);
      case '7':
      data = LCD_WRITE|(NUM_7_LCD<<8);
      case '8':
      data = LCD_WRITE|(NUM_8_LCD<<8);
      case '9':
      data = LCD_WRITE|(NUM_9_LCD<<8);
      default:
      trace_printf("something went wrong!");
    }

    IOOUT = data;
    CNT = TIMER;
    while((0 < (TIMER-CNT))||((TIMER-CNT) < (SystemCoreClock/20000))){
      //delay for 50 microseconds
    }
    IOOUT = LCD_READBF;
    while ((IOIN & LCD_BUSY) != 0) {
      //wait until busy signal is active
    }
  }
}

void lcdStart(){
  IOOUT = LCD_FUNCTSET;
  CNT = TIMER;
  while((0 < (TIMER-CNT))||((TIMER-CNT) < (SystemCoreClock/10000))){
    //delay for 100 microseconds
  }

  IOOUT = LCD_PCONT;
  CNT = TIMER;
  while((0 < (TIMER-CNT))||((TIMER-CNT) < (SystemCoreClock/25000))){
    //delay for 40 microseconds
  }

  IOOUT = LCD_CLEAR;
  CNT = TIMER;
  while((0 < (TIMER-CNT))||((TIMER-CNT) < (SystemCoreClock/625))){
    //delay for 1.6 milliseconds
  }

  IOOUT = LCD_EMS;
  CNT = TIMER;
  while((IOIN & LCD_BUSY) != 0){
    //wait until the busy flag is gone
  }
  lcdPrint("LCD ON","NICE");
}


#endif /* HARDLCD_H_ */
