

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include "c8051F340.h"                 // SFR declarations
#include <stdio.h>

//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F34x
//-----------------------------------------------------------------------------

sfr16 TMR2RL   = 0xca;                 // Timer2 reload value 
sfr16 TMR2     = 0xcc;                 // Timer2 counter
sfr16 ADC0     = 0xbd;                 // ADC0 result

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSCLK       12000000          // SYSCLK frequency in Hz
#define BAUDRATE     115200            // Baud rate of UART in bps

sbit LED = P2^2;                       // LED='1' means ON
code int pis=1;
code char tab4[]={0x70,0x68,0x58,0x38};
code float rtab[]={0,14.8406,29.2571,
43.2676,55.5438,68.8285,81.7537,94.3338,110.194,
122.031,133.565,144.808,154.687,165.406,175.864,
186.07,195.048,204.8,214.326,223.632,231.827,240.738,
249.451,257.971,268.772,276.876,284.807,292.571,299.421,
306.884,314.194,321.356,334.574,341.333,347.961,354.462,
360.206,366.474,372.625,378.662,386.344,392.131,397.813,
403.394,408.332,413.728,419.03,424.241,428.855,433.898,
438.857,443.733,448.053,452.778,457.426,461.998,467.834,
472.241,476.579,480.849,484.635,488.781,492.863,496.884};
   volatile unsigned char position[2][8]={0};
//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void SYSCLK_Init (void);
void PORT_Init (void);
void Timer2_Init(void);
void ADC0_Init(void);
void UART0_Init (void);
unsigned int  decide();               //data change?
unsigned char deal(int kk);                   //a function deal with nonlinear data
void execute(unsigned int count);                                //打开第count路电阻
//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------

void main (void) 
{
   PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer 
                                       // enable)

   SYSCLK_Init ();                     // Initialize system clock to 
                                       // 24.5MHz
   PORT_Init ();                       // Initialize crossbar and GPIO
   Timer2_Init();                      // Init Timer2 to generate 
                                       // overflows to trigger ADC
   UART0_Init();                       // Initialize UART0 for printf's
   ADC0_Init();                        // Initialize ADC0

   EA = 1;							         // enable global interrupts
   while (1) {                         // spin forever
   }
}

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// This routine initializes the system clock to use the internal 12MHz 
// oscillator as its clock source.  Also enables missing clock detector reset.
//
//-----------------------------------------------------------------------------
void SYSCLK_Init (void)
{
   OSCICN = 0x83;                      // configure internal oscillator for
                                       // 12MHz / 1
   RSTSRC = 0x04;                      // enable missing clock detector
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure the Crossbar and GPIO ports.
// P0.4 - UART TX (push-pull)
// P0.5 - UART RX 
// P1.1 - ADC0 analog input
// P2.2 - LED (push-pull)
// 
//-----------------------------------------------------------------------------
void PORT_Init (void)
{
   XBR0     = 0x01;                    // Enable UART0
   XBR1     = 0xC0;                    // Enable crossbar and weak pull-ups
   P0MDOUT |= 0x10;                    // Set TX pin to push-pull
   P2MDOUT |= 0x04;                    // enable LED as a push-pull output
   P1MDIN &= ~0x02;                    // set P1.1 as an analog input
   P3MDOUT |= 0x7F;	
}

//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure Timer2 to 16-bit auto-reload and generate an interrupt at 100uS 
// intervals.  Timer 2 overflow automatically triggers ADC0 conversion.
// 
//-----------------------------------------------------------------------------

void Timer2_Init (void)
{
   TMR2CN  = 0x00;                     // Stop Timer2; Clear TF2;
                                       // use SYSCLK as timebase, 16-bit 
                                       // auto-reload
   CKCON  |= 0x10;                     // select SYSCLK for timer 2 source
   TMR2RL  = 65535 - (SYSCLK / 40000); // init reload value for 20uS	!!!!!!!!!!!!!!!!!!!!!!!!!!!
   TMR2    = 0xffff;                   // set to reload immediately
   TR2     = 1;                        // start Timer2
}

//-----------------------------------------------------------------------------
// ADC0_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configures ADC0 to make single-ended analog measurements on pin P1.1
//  
//-----------------------------------------------------------------------------

void ADC0_Init (void)
{
   ADC0CN = 0x02;                      // ADC0 disabled, normal tracking, 
                                       // conversion triggered on TMR2 overflow

   REF0CN = 0x0a;                      // Enable on-chip VREF and buffer

   AMX0P = 0x13;                       // ADC0 positive input = P1.1
   AMX0N = 0x1F;                       // ADC0 negative input = GND
                                       // i.e., single ended mode

   ADC0CF = ((SYSCLK/3000000)-1)<<3;   // set SAR clock to 3MHz

   ADC0CF |= 0x00;                     // right-justify results 

   EIE1 |= 0x08;                       // enable ADC0 conversion complete int.

   AD0EN = 1;                          // enable ADC0
}

//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure the UART0 using Timer1, for <BAUDRATE> and 8-N-1.
//
//-----------------------------------------------------------------------------

void UART0_Init (void)
{
   SCON0 = 0x10;                       // SCON0: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled
                                       //        ninth bits are zeros
                                       //        clear RI0 and TI0 bits
   if (SYSCLK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSCLK/BAUDRATE/2);
      CKCON |=  0x08;                  // T1M = 1; SCA1:0 = xx
   } else if (SYSCLK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSCLK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   } else if (SYSCLK/BAUDRATE/2/256 < 48) {
      TH1 = -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   } else {
      while (1);                       // Error.  Unsupported baud rate
   }

   TL1 = TH1;                          // init Timer1
   TMOD &= ~0xf0;                      // TMOD: timer 1 in 8-bit autoreload
   TMOD |=  0x20;
   TR1 = 1;                            // START Timer1
   TI0 = 1;                            // Indicate TX0 ready
}

//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// ADC0_ISR
//-----------------------------------------------------------------------------
// 
// This ISR averages 2048 samples then prints the result to the terminal.  The 
// ISR is called after each ADC conversion which is triggered by Timer2.
//
//-----------------------------------------------------------------------------

void ADC0_ISR (void) interrupt 10
{

   static unsigned long accumulator = 0;     // Accumulator for averaging
   static unsigned int measurements = 100;  // Measurement counter
   static unsigned int mm=0;
   static unsigned char change=0;
   static unsigned char over=0;
   unsigned char ii;
   int result=0;
 //  unsigned long mV;                         // Measured voltage in mV

   AD0INT = 0;                               // Clear ADC0 conv. complete flag
      TR2     = 0;
   accumulator += ADC0;
   measurements--;

   if(measurements==0)
   {  
      measurements = 100;
      result = accumulator / 100;
      accumulator=0;
   //   mV =  result * 3300 / 1023;   
 //     printf("P1.1 voltage: %ld mV\n",mV);
//	  putchar(deal(result));
				  position[change][mm]= deal(result);
            if ((mm==(8*pis-1)))                                     //测得一组32个数据后
		 {
			 mm=0;    
			 over=1;                                        //重新测一组
			 if (decide()==1)                                      //如果有就发送数据
			 {
				 putchar(0xaa);
				 for(ii=0;ii<8*pis;ii++)
				 {
					 putchar(position[change][ii]);
				 }
				 putchar(0x55);                      //多发一个数据确认一组结束
				 if (change==1)
				 {
					 change=0;
				 }
				 else
				 {
					 change=1;
				 }
			 }
		 }
		 if (over==0)
		 	 mm ++;
		 if (over==1)		 
		      over=0;
			 execute(mm);
   }
   LED=~LED;                           // Toggle LED
        TR2  = 1;
}

 unsigned int decide()
{
	unsigned int ii=0;
	unsigned int mtn=0; 
	for(ii=0;ii<(8*pis);ii++)
	{
		if (position[0][ii]!=position[1][ii])
		{
			mtn= 1;
		}
	}
    return mtn;
}

 unsigned char deal(int kk)                   //a function deal with nonlinear data
{
    unsigned char key_deal=0;
	unsigned char ii=0; 
	unsigned char iq=0;                              
 	for (ii=0;ii<63;ii++)
 	{

       if ((kk-rtab[ii])>=0 && (kk-rtab[ii+1])<0)
       {
	       iq=ii;
	   }

 	}
     if((kk-rtab[63])>=0)
	       iq=63;

	if(iq==63)
	    key_deal=iq;
	else
	{
    if ((kk-rtab[iq])<=(rtab[iq+1]-kk))
       {
	       key_deal=iq;
	   }
	  else
	  {
		  key_deal=iq+1;
	  }
    }	 
  return key_deal;
 }
 void execute(unsigned int count)
{
	unsigned int i;
	unsigned int j;
	i=count & 0x07;
	j=(count & 0x18) >> 3;
	
	P3&=0x78;
	P3|=i;
	
	P3&=0x07;
	P3|=tab4[j];		
}
//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
