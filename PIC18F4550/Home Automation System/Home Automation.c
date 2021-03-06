//program


#pragma config PLLDIV = 5
#pragma config CPUDIV = OSC1_PLL2
#pragma config USBDIV = 2

#pragma config FOSC = INTOSCIO_EC
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOR = OFF
#pragma config BORV = 3
#pragma config VREGEN = OFF
#pragma config WDT = OFF
#pragma config WDTPS = 32768
#pragma config CCP2MX = ON
#pragma config PBADEN = OFF
#pragma config LPT1OSC  = OFF
#pragma config MCLRE = OFF
#pragma config STVREN = ON
#pragma config LVP = OFF
#pragma config ICPRT = OFF
#pragma config XINST = OFF
#pragma config DEBUG = OFF
#pragma config CP0 = OFF,CP1 = OFF,CP2 = OFF,CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF,WRT1 = OFF,WRT2 = OFF,WRT3 = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF
#pragma config EBTR0 = OFF,EBTR1 = OFF,EBTR2 = OFF,EBTR3 = OFF
#pragma config EBTRB = OFF

#include <p18f4550.h>

typedef unsigned char BYTE;
//#define FLEX0 PORTAbits.RA0
//#define LED0 PORTBbits.RB3
//#define LED1 PORTBbits.RB2
//#define LED2 PORTBbits.RB1
//#define LED3 PORTBbits.RB0
//#define LED4 PORTDbits.RD7
//#define LED5 PORTDbits.RD6
//#define LED6 PORTDbits.RD5
//#define LED7 PORTDbits.RD4


#define LED0 PORTBbits.RB0
#define LED1 PORTBbits.RB1

void delay(void);
void showByteOnLEDs(BYTE byteToShow);

void main(void){
	int a ;
	unsigned int t;
	//setting registers

	OSCCONbits.IRCF2 = 1;//IRCF  = Set internal clock speed to 8mhz so set IRCF0,1,2 bits in ISCCOn register to 1
	OSCCONbits.IRCF1 = 1;
	OSCCONbits.IRCF0 = 1;

	TRISAbits.RA0 = 1; //set analog pin as input and led as output
	TRISAbits.RA1 = 1;

	TRISBbits.RB0 = 0; //set led as output
	TRISBbits.RB1 = 0;
	//TRISBbits.RB2 = 0;
	//TRISBbits.RB3 = 0;
	//TRISDbits.RD7 = 0;
	//TRISDbits.RD6 = 0;
	//TRISDbits.RD5 = 0;
	//TRISDbits.RD4 = 0;


	//Making AN0/RA0 as Analog Input
	ADCON0bits.CHS3 = 0; //setting analog channel to RA0
	ADCON0bits.CHS2 = 0;
	ADCON0bits.CHS1 = 0;
	ADCON0bits.CHS0 = 0;

	ADCON1bits.VCFG1 = 0; //setting range for the analog input here vcfg1 = lowerbound = GND, vcf0 = upperbound = 5v from vdd(power from usb) // set A/D converter negative reference Ground as low
	ADCON1bits.VCFG0 = 0; // set A/D converter positive reference VCC as high

	ADCON1bits.PCFG3 = 1; //setting which input pins will acts as analog or digital in this case only 1 analog input so this combination will make AN0+AN1 as analog and rest AN3-12 to digital
	ADCON1bits.PCFG2 = 1;
	ADCON1bits.PCFG1 = 0;
	ADCON1bits.PCFG0 = 1;

	ADCON2bits.ADFM = 0; //ADC results will be left justified (8 bit result) in ADRESH

	ADCON2bits.ACQT2 = 1; // set adc aquision time
	ADCON2bits.ACQT1 = 0;
	ADCON2bits.ACQT0 = 1;

	ADCON2bits.ADCS2 = 1; // set adc conversion clock
	ADCON2bits.ADCS1 = 0;
	ADCON2bits.ADCS0 = 1;

	ADCON0bits.ADON = 1; // turn adc on



	while(1){

		 //LDR input in RA0/AN0
		 //Making AN0/RA0 as Analog Input
			ADCON0bits.CHS3 = 0; //setting analog channel to RA0
			ADCON0bits.CHS2 = 0;
			ADCON0bits.CHS1 = 0;
			ADCON0bits.CHS0 = 0;

		 	ADCON0bits.GO_DONE = 1; // perform A-D Conversion
		 	while(ADCON0bits.GO_DONE == 1){}; // wait for conversion
		  	a = ADRESH;
		  	if(a < 100)LED0 = 1; // Light sensor trigger value
		  	else LED0 = 0 ;



		  //Temparature Sensor input in RA1/AN1
		  //Making AN1/RA1 as Analog Input
			ADCON0bits.CHS3 = 0; //setting analog channel to RA1
			ADCON0bits.CHS2 = 0;
			ADCON0bits.CHS1 = 0;
			ADCON0bits.CHS0 = 1;

			delay();

			ADCON0bits.GO_DONE = 1; // perform A-D Conversion
		 	while(ADCON0bits.GO_DONE == 1){}; // wait for conversion
		  	a = ADRESH;
		  	t = (((int)a/255.0) * 500);
			if(t > 26 )LED1 = 1; //Temparature sensor trigger value
		  	else LED1 = 0 ;



	}



}
void delay(void){
	int i,j;
	for(i = 0; i < 1000; i ++)for(j = 0; j < 2; j++);
}
