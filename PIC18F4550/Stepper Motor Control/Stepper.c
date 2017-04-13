#pragma config VREGEN = OFF         // Voltage regulator USB , is Suspended
#pragma config WDT = OFF                // Watchdog timer is suspended
#pragma config PLLDIV = 5                // Internal Oscillator engaged
#pragma config MCLRE = OFF
#pragma config WDTPS = 32768
#pragma config CCP2MX = ON
#pragma config PBADEN = OFF
#pragma config CPUDIV = OSC1_PLL2
#pragma config USBDIV = 2
#pragma config FOSC = INTOSCIO_EC
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOR = OFF
#pragma config BORV = 3
#pragma config LPT1OSC = OFF
#pragma config STVREN = ON
#pragma config LVP = OFF
#pragma config ICPRT = OFF
#pragma config XINST = OFF
#pragma config DEBUG = OFF
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config CPB = OFF                                                                              // CPB off
#pragma config CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
#pragma config EBTRB = OFF
#include <p18f4550.h>
#define _XTAL_FREQ 2000000 //20 for the Delays 
#define port_d_op TRISD = 0;
#define port_b_op TRISB = 0;

void delay(void);void delay1(void);

void fullstepping (void)
{
    int c;
    PORTB = 0b00000111; delay();
    PORTB = 0b00000100;  delay();

 for(c=0;c<10;c++)
   {
     PORTD = 0b11000000;  delay();
     PORTD = 0b01100000;  delay();
     PORTD = 0b00110000;  delay();
     PORTD = 0b10010000;  delay();
   }
}

void singlestepping (void)
{
    int a;

    PORTB = 0b00000111; delay();
    PORTB = 0b00000001; delay();

 for(a=0;a<10;a++)
   {
     PORTD = 0b10000000;    delay();
     PORTD = 0b01000000;    delay();
     PORTD = 0b00100000;    delay();
     PORTD = 0b00010000;    delay();
   }
}

void stop(){
	 PORTD = 0b00000000;  delay();
     PORTD = 0b00000000;  delay();
     PORTD = 0b00000000;  delay();
     PORTD = 0b00000000;  delay();
}

void main (void)
{
    port_d_op; // Make Pord D output
    port_b_op; // make Port B output

    while(1) // Steppings
    {
       
        int k;
        for(k = 0; k < 5; k++)fullstepping();
        
        stop();
        delay1();
        
        
        
    }
}

void delay(){
	int k;
	for(k = 0; k < 100; k++){}
}


void delay1(){
	int k;
	for(k = 0; k < 2000; k++){}
}
