/**********************************************************************************/
/*    Demo program for:								  */
/*	  Board: MSP430-PIR     						  */
/*    Manufacture: OLIMEX                                                   	  */
/*	  COPYRIGHT (C) 2011							  */
/*    Designed by: Engineer Penko T. Bozhkov                                      */
/*    Module Name    :  main module                                               */
/*    File   Name    :  main.c                                                    */
/*    Revision       :  initial                                                   */
/*    Date           :  08.02.2011                                                */
/*    Built with IAR Embedded Workbench Version: 4.21                             */
/**********************************************************************************/
#include  <intrinsics.h>
#include  <msp430x20x3.h>

#define LED BIT7 
#define LED_OUT P1OUT
#define LED_DIR P1DIR
#define TAMPER BIT4
#define REED BIT0

unsigned int blink = 0;

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
  LED_DIR |= (LED); // Set P1.0 and P1.6 to output direction
  LED_OUT &= ~(LED); // Set the LEDs off
  P1REN |= TAMPER; //Enables pullup
  P1OUT |= TAMPER; //pullup HIGH
  P1IES |= TAMPER; // Triggers when you PRESS the button
  P1REN |= REED;
  P1OUT |= REED;
  P1IES &= ~ REED;// Triggers when you RELEASE the button
  P1IE |= TAMPER; //Enables the selector-mask for generating interrupts on the relevant pin
  P1IE |= REED;
  __enable_interrupt(); // Interrupts get enabled *here* - they were disabled thus far..
  
  for (;;)
  {    
    if(blink > 0)
    {
      P1OUT ^= (LED);
      __delay_cycles(100000); // SW Delay of 10000 cycles at 1Mhz
    }
  }  
} 

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  blink ^= 0x01;
  P1IFG &= ~REED; // P1.3 IFG cleared
  P1IFG &= ~TAMPER; // P1.3 IFG cleared
  LED_OUT &= ~(LED); // Clear the LEDs so they start in OFF state  
}