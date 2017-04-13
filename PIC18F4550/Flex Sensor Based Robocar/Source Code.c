//program

#pragma config PLLDIV = 5                       //this does not matter since we are using the internal clock
#pragma config CPUDIV = OSC1_PLL2				//this does not matter since we are using the internal clock	
#pragma config USBDIV = 2						//this does not matter since we are using the internal clock

												//internal clock, pin #14 (RA6) as I/O pin, pin#13 unused
#pragma config FOSC = INTOSCIO_EC				//if OSCCON is left as default , clock speed will be 1 MHz

												// now for the other less confusing options...
#pragma config FCMEN = OFF						//fail safe monitor disabled					
#pragma config IESO = OFF						//internal  / external osc switch over bit disabled
#pragma config PWRT = OFF						// power up timer disabled
#pragma config BOR = OFF						//brown out reset disabled in hardware and software
#pragma config BORV = 3							//brown out reset voltage bits, does not matter since brown-out voltage is disabled
#pragma config VREGEN = OFF						//USB voltage regulator, would need to turn on if using internal USB voltage regulator for USB comm.
#pragma config WDT = OFF						//watchdog timer disabled
#pragma config WDTPS = 32768					//watchdog timer postscale , does not matter since watchdog timer is disabled
#pragma config CCP2MX = ON						//use RC1(PIN#16) as CCP2 MUX (this is the default pin for CCP2 MUX)
#pragma config PBADEN = OFF						//RB0 , RB1 , RB2 , RB3 & RB4 are configured as digital I/O on reset
#pragma config LPT1OSC  = OFF					//disable low power option for timer 1 (timer 1 in regular mode)
#pragma config MCLRE = OFF						//master clear disabled, pin#1 is for VPP and / or RE3 use
#pragma config STVREN = ON						//stack full / underflow will cause reset
#pragma config LVP = OFF						//single-supply ICSP disabled
#pragma config ICPRT = OFF						//in-circuit debug / programming port (ICPORT) disabled, this feature is not available on 40 pin DIP package
#pragma config XINST = OFF						//instruction set externsion and index addressing mode disabled
#pragma config DEBUG = OFF						//background debugger disabled, RA6 & RB7 configured as general purpose I/O pins
#pragma config CP0 = OFF,CP1 = OFF,CP2 = OFF,CP3 = OFF			//code protection bits off
#pragma config CPB = OFF						//boot block code protection off
#pragma config CPD = OFF						//data EEPROM code protection off
#pragma config WRT0 = OFF,WRT1 = OFF,WRT2 = OFF,WRT3 = OFF		//write protection bits off
#pragma config WRTC = OFF						//config registers write protection off
#pragma config WRTB = OFF						//boot block is not write protected
#pragma config WRTD = OFF						//data EEPROM is not write protected
#pragma config EBTR0 = OFF,EBTR1 = OFF,EBTR2 = OFF,EBTR3 = OFF	//table read protection bits off
#pragma config EBTRB = OFF						//boot block table read protection off


#include <p18f4550.h>

typedef unsigned char BYTE;
#define FLEX0 PORTAbits.RA0
#define M1_0 PORTBbits.RB0
#define M1_1 PORTBbits.RB1
#define M2_0 PORTBbits.RB2
#define M2_1 PORTBbits.RB3


void delay(void);
void workWithLEDS(BYTE data);

void main(void){
	unsigned char a ;
	//setting registers
	
	OSCCONbits.IRCF2 = 1;//IRCF  = Set internal clock speed to 8mhz so set IRCF0,1,2 bits in ISCCOn register to 1
	OSCCONbits.IRCF1 = 1;
	OSCCONbits.IRCF0 = 1;
	
	TRISAbits.RA0 = 1; //set analog pin as input and led as output
	TRISBbits.RB0 = 0;
	TRISBbits.RB1 = 0;
	TRISBbits.RB2 = 0;
	TRISBbits.RB3 = 0;
	
	ADCON0bits.CHS3 = 0; //setting analog channel to RA0
	ADCON0bits.CHS2 = 0;
	ADCON0bits.CHS1 = 0;
	ADCON0bits.CHS0 = 0;
	
	ADCON1bits.VCFG1 = 0; //setting range for the analog input here vcfg1 = lowerbound = GND, vcf0 = upperbound = 5v from vdd(power from usb)
	ADCON1bits.VCFG0 = 0;
	
	ADCON1bits.PCFG3 = 1; //setting which input pins will acts as analog or digital in this case only 1 analog input so this combination will make AN0 as analog and rest AN1-12 to digital   
	ADCON1bits.PCFG2 = 1;
	ADCON1bits.PCFG1 = 1;
	ADCON1bits.PCFG0 = 0;
	
	ADCON2bits.ADFM = 0; //ADC results will be left justified (8 bit result) in ADRESH
	
	ADCON2bits.ACQT2 = 1; // set adc aquision time
	ADCON2bits.ACQT1 = 0;
	ADCON2bits.ACQT0 = 1;
	
	ADCON2bits.ADCS2 = 1; // set adc conversion clock
	ADCON2bits.ADCS1 = 0;
	ADCON2bits.ADCS0 = 1;
	
	ADCON0bits.ADON = 1; // turn adc on
	
			
	while(1){
		  	int right_side = 0, left_side = 0;
		  	
		  	// Sensor input in RA0/AN0
			ADCON0bits.CHS3 = 0; //setting analog channel to RA0
			ADCON0bits.CHS2 = 0;
			ADCON0bits.CHS1 = 0;
			ADCON0bits.CHS0 = 0;
		 
		 	ADCON0bits.GO_DONE = 1; // perform A-D Conversion
		 	while(ADCON0bits.GO_DONE == 1){}; // wait for conversion
		  	a = ADRESH;
		  	if(a > 70)left_side = 1; // Light sensor trigger value
		  	else left_side = 0 ;
		  
		  	
		  
		  // Sensor input in RA1/AN1
		  //Making AN1/RA1 as Analog Input
			ADCON0bits.CHS3 = 0; //setting analog channel to RA1
			ADCON0bits.CHS2 = 0;
			ADCON0bits.CHS1 = 0;
			ADCON0bits.CHS0 = 1;
			
			//delay();
			
			ADCON0bits.GO_DONE = 1; // perform A-D Conversion
		 	while(ADCON0bits.GO_DONE == 1){}; // wait for conversion
		  	a = ADRESH;
			if(a > 70 )right_side = 1; //Temparature sensor trigger value
		  	else right_side = 0 ;
			
			
			/*
			b1 effecting right side; b2 effecting left side
			*/
			
			
			
			
			//right motor M1 left motor M2
			if(right_side == 1 && left_side == 1)  // this logic for move forward // two wheel move samve
			{
				M1_0 = 0;
				M1_1 = 1;
				M2_0 = 0;
				M2_1 = 1;
			}
			else if(right_side == 1 && left_side == 0)  // this logic for move right // left_wheel on // right_wheel off
			{
				M1_0 = 0;
				M1_1 = 0;
				M2_0 = 0;
				M2_1 = 1;
			}
			else if(right_side == 0 && left_side == 1)  // this logic for move leftt // left_wheel off // right_wheel on
			{
				M1_0 = 0;
				M1_1 = 1;
				M2_0 = 0;
				M2_1 = 0;
			}
			else if(right_side == 0 && left_side == 0)  // this logic stop // left_wheel off // right_wheel off
			{
				M1_0 = 0;
				M1_1 = 0;
				M2_0 = 0;
				M2_1 = 0;
			}	
					  
		  //showByteOnLEDs(ADRES);		  
		  
	}

	
}



void delay(void){
	int i,j;
	for(i = 0; i < 1000; i ++)for(j = 0; j < 2; j++);
}