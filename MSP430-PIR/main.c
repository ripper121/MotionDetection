#include  <msp430x20x3.h>

#define   LED_OUT         BIT5              // Bit location for LED
#define   SENSOR_PWR      BIT7              // Bit location for power to sensor
#define   THRESHOLD       50                // Threshold for motion

static unsigned int result_old = 0;         // Storage for last conversion
char LED_ENABLE = 1;                        // LED control



#define Number_of_Bytes  1                  // **** How many bytes?? ****

void Setup_USI_Slave(void);

char MST_Data = 0x00;                          // Variable for received data
char SLV_Data = 0x00;
char SLV_Addr = 0x90;                       // Address is 0x48<<1 for R/W
int I2C_State, Bytecount, transmit = 0;     // State variables

void Data_RX(void);
void TX_Data(void);


void main(void)
{
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL; // ACLK/32768, int timer: ~10s
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
  DCOCTL = CALDCO_1MHZ;
  BCSCTL1 |= DIVA_2;                        // ACLK = VLO/4
  BCSCTL3 |= LFXT1S_2;
  
  P1OUT = 0x10;                             // P1OUTs
  P1SEL = 0x08;                             // Select VREF function
  P1DIR = 0xEF;                             // Unused pins as outputs
  P1OUT |= LED_OUT;
  P2OUT = 0x00 + SENSOR_PWR;                // P2OUTs
  P2SEL &= ~SENSOR_PWR;                     // P2.7 = GPIO
  P2DIR = 0xff;                             // Unused pins as outputs
  
  SD16CTL = SD16VMIDON + SD16REFON + SD16SSEL_1;// 1.2V ref, SMCLK
  SD16INCTL0 = SD16GAIN_4 + SD16INCH_4;     // PGA = 4x, Diff inputs A4- & A4+
  SD16CCTL0 =  SD16SNGL + SD16IE;           // Single conversion, 256OSR, Int enable
  SD16CTL &= ~SD16VMIDON;                   // VMID off: used to settle ref cap
  SD16AE = SD16AE1 + SD16AE2;               // P1.1 & P1.2: A4+/- SD16_A inputs
  
  // Wait for PIR sensor to settle: 1st WDT+ interval
  P1SEL |= LED_OUT;                         // Turn LED on with ACLK (for low Icc)
  while(!(IFG1 & WDTIFG));                  // ~5.4s delay: PIR sensor settling
  P1SEL &= ~LED_OUT;                        // Turn LED off with ACLK (for low Icc)
  
  // Reconfig WDT+ for normal operation: interval of ~341msec
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1;// ACLK/512, int timer: 341msec
  
  if (CALBC1_1MHZ==0xFF)		    // If calibration constants erased
  {											
    while(1);                               // do not load, trap CPU!!	
  }
  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
  DCOCTL = CALDCO_1MHZ;
  Setup_USI_Slave();
  
  IE1 |= WDTIE;                             // Enable WDT interrupt
  
  _BIS_SR(LPM3_bits + GIE);                 // Enter LPM3 with interrupts
}

/******************************************************
// SD16_A interrupt service routine
******************************************************/
#pragma vector = SD16_VECTOR
__interrupt void SD16ISR(void)
{ unsigned int result_new;

SD16CTL &= ~SD16REFON;                    // Turn off SD16_A ref
result_new = SD16MEM0;                    // Save result (clears IFG)

if (result_new > result_old)              // Get difference between samples
result_old = result_new - result_old;
else
result_old = result_old - result_new;

if (result_old > THRESHOLD)               // If motion detected...
P1OUT |= LED_OUT;                   // Turn LED on

result_old = SD16MEM0;                    // Save last conversion

__bis_SR_register_on_exit(SCG1+SCG0);     // Return to LPM3 after reti
}

/******************************************************
// Watchdog Timer interrupt service routine
******************************************************/
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
  if (!(P1OUT & LED_OUT))                   // Has motion already been detected?
  {
    SD16CTL |= SD16REFON;                   // If no, turn on SD16_A ref
    SD16CCTL0 |= SD16SC;                    // Set bit to start new conversion
    __bic_SR_register_on_exit(SCG1+SCG0);   // Keep DCO & SMCLK on after reti
  }
  else
    P1OUT &= ~LED_OUT;                      // If yes, turn off LED, measure on next loop
}

















//******************************************************************************
// USI interrupt service routine
// Rx bytes from master: State 2->4->6->8 
// Tx bytes to Master: State 2->4->10->12->14
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USI_VECTOR
__interrupt void USI_TXRX (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USI_VECTOR))) USI_TXRX (void)
#else
#error Compiler not supported!
#endif
{
  if (USICTL1 & USISTTIFG)                  // Start entry?
  {
    I2C_State = 2;                          // Enter 1st state on start
  }
  
  switch(__even_in_range(I2C_State,14))
  {
  case 0:                               // Idle, should not get here
    break;
    
  case 2: // RX Address
    USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, RX address
    USICTL1 &= ~USISTTIFG;        // Clear start flag
    I2C_State = 4;                // Go to next state: check address
    
    break;
    
  case 4: // Process Address and send (N)Ack
    if (USISRL & 0x01){            // If master read...
      SLV_Addr = 0x91;             // Save R/W bit
      transmit = 1;}
    else{transmit = 0;
    SLV_Addr = 0x90;}
    USICTL0 |= USIOE;             // SDA = output
    if (USISRL == SLV_Addr)       // Address match?
    {
      USISRL = 0x00;              // Send Ack
      if (transmit == 0){ 
        I2C_State = 6;}           // Go to next state: RX data
      if (transmit == 1){  
        I2C_State = 10;}          // Else go to next state: TX data
    }
    else
    {
      USISRL = 0xFF;              // Send NAck
      I2C_State = 8;              // next state: prep for next Start
    }
    USICNT |= 0x01;               // Bit counter = 1, send (N)Ack bit
    break;
    
  case 6: // Receive data byte
    Data_RX();
    break;  
    
  case 8:// Check Data & TX (N)Ack
    USICTL0 |= USIOE;             // SDA = output
    MST_Data = USISRL;            //Received Data
    if (MST_Data % 2)
    {
      P1OUT |= 0x01;
    }
    else
    {
      P1OUT &= ~0x01;
    }
    if (Bytecount <= (Number_of_Bytes-2))// If not last byte
    {
      USISRL = 0x00;              // Send Ack
      I2C_State = 6;              // Rcv another byte
      Bytecount++;
      USICNT |= 0x01;             // Bit counter = 1, send (N)Ack bit
    }
    else                          // Last Byte
    {
      USISRL = 0xFF;              // Send NAck
      USICTL0 &= ~USIOE;            // SDA = input
      SLV_Addr = 0x90;              // Reset slave address
      I2C_State = 0;                // Reset state machine
      Bytecount =0;                 // Reset counter for next TX/RX
    }
    
    
    break;
    
  case 10: // Send Data byte
    TX_Data();
    break;
    
  case 12:// Receive Data (N)Ack
    USICTL0 &= ~USIOE;            // SDA = input
    USICNT |= 0x01;               // Bit counter = 1, receive (N)Ack
    I2C_State = 14;               // Go to next state: check (N)Ack
    break;
    
  case 14:// Process Data Ack/NAck
    if (USISRL & 0x01)               // If Nack received...
    {
      USICTL0 &= ~USIOE;            // SDA = input
      SLV_Addr = 0x90;              // Reset slave address
      I2C_State = 0;                // Reset state machine
      Bytecount = 0;
      // LPM0_EXIT;                  // Exit active for next transfer
    }
    else                          // Ack received
    {
      TX_Data();                  // TX next byte
    }
    break;
    
  }
  USICTL1 &= ~USIIFG;                       // Clear pending flags
}

void Data_RX(void){
  
  USICTL0 &= ~USIOE;            // SDA = input
  USICNT |=  0x08;              // Bit counter = 8, RX data
  I2C_State = 8;                // next state: Test data and (N)Ack
}

void TX_Data(void){
  USICTL0 |= USIOE;             // SDA = output              
  SLV_Data = MST_Data;          //ECHO Test TX=RX
  USISRL = SLV_Data++;
  USICNT |=  0x08;              // Bit counter = 8, TX data
  I2C_State = 12;               // Go to next state: receive (N)Ack
}

void Setup_USI_Slave(void){
  P1OUT = 0xC0;                             // P1.6 & P1.7 Pullups
  P1REN |= 0xC0;                            // P1.6 & P1.7 Pullups
  
  USICTL0 = USIPE6+USIPE7+USISWRST;         // Port & USI mode setup
  USICTL1 = USII2C+USIIE+USISTTIE;          // Enable I2C mode & USI interrupts
  USICKCTL = USICKPL;                       // Setup clock polarity
  USICNT |= USIIFGCC;                       // Disable automatic clear control
  USICTL0 &= ~USISWRST;                     // Enable USI
  USICTL1 &= ~USIIFG;                       // Clear pending flag
  
  transmit = 0;
  __enable_interrupt();  
}
