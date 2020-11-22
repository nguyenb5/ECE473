// Lab4
// Expected Connections:
// Bargraph board           Mega128 board 
// --------------      ----------------------    
//     reglck            PORTD bit 2                     
//     srclk             PORTB bit 1 (sclk)
//     sdin              PORTB bit 2 (mosi)
//     oe_n                   ground
//     gnd2                   ground
//     vdd2                     vcc
//     sd_out               no connect

// 
/* Encoder board */
// 	SH/LD		PORTE bit 6
// 	CLK_INH	PORTE bit 7
//	SCK		PORTB bit 1 (sclk)
//	SER_OUT	PORTB bit 3 (miso)

/* ADC CdS cell */
// Photocell 	PORTF bit 7

/* AudioBoard */
// Audio_from_atmega		PORTD bit 4
// Atmega_vol_control		PORTE bit 3

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "hd44780.h"

uint8_t indexToLed[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80,
			0x98, 0xFF};
uint8_t brightness[10] = {0xD0, 0xD0, 0xB5, 0xA1, 0x8D, 0x79, 0x65, 0x51, 0x3D, 0x15, 0x15};

/** Global Vars **/
volatile uint8_t newEncodeData;
volatile uint32_t current_second; 
volatile bool clk_mode;
volatile uint8_t stateEncoder1;
volatile uint8_t stateEncoder2;
volatile uint8_t dot;
volatile uint16_t lastADCread = 217; 
bool button_flag[2] = {0};

volatile uint8_t inc_dec;
int i;	//dummy var

char     lcd_str_h[16];  //holds string to send to lcd  
char     lcd_str_l[16];  //holds string to send to lcd 
div_t    fp_adc_result, fp_low_result;  //double fp_adc_result; 
/** End Global Vars **/

//Initialization
void spi_init(void);
void clock_config_init(void);
void IO_config_init(void);
//ADC stuff
void ADC_init(void);
uint16_t CdS_read(void);
void setBrightness(uint8_t value);
//Encoder stuff
int dirOfEncoder(void);
void updateSPI(void);
//LED stuff
void processLEDbrightness(uint16_t ADC_value);
void update7Segment(uint16_t number, uint8_t dot);
uint16_t second_to_min_hour(uint32_t second, bool mode);

void spi_init(void){
	/* Run this code before attempting to write to the LCD.*/
 DDRF  |= 0x08;  //port F bit 3 is enabling for LCD
 PORTF &= 0xF7;  //port F bit 3 is initially low

 DDRB  |= 0x07;  //Turn on SS_n, MOSI, SCLK. SS_n must be out for MSTR mode
//see: /$install_path/avr/include/avr/iom1z28.h for bit definitions   

 //Master mode, Clock=clk/4, Cycle half phase, Low polarity, MSB first
 SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
 SPSR=(1<<SPI2X);           //SPI at 2x speed (8 MHz)  
}

/*

*/
void clock_config_init(void){

	/* Timer2 OC2 PWM for PB7 */
	//Enable fast PWM, non-inverting output mode
  	//64 prescaler (goal is 967Hz)
  	TCCR2 = (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<CS21) | (1<<CS20);
  	//Default PWM value of half brightness
  	//OCR2 set the BOTTOM. Output flip when pass BOTTOM and TOP
  	OCR2 = 0xE1; //Half brightness at start up
	
	/* Config timer 0 to keep track of seconds*/
	TCCR0 |= (1<<CS00);  //normal mode, no prescaling
	ASSR  |= (1<<AS0);   //use ext oscillator
	TIMSK |= (1<<TOIE0); //allow interrupts on overflow
	
	/* Config timer 3 */
}

void IO_config_init(void){
	DDRE |= (1<<PE3);		//Volume control pin for audio
	DDRD |= (1<<PD4);	//Audio out for audio
	
	DDRA = 0xFF;		//PortA to all output for 4 digits 7 seg LED
	DDRB |= 0xF0;		//PortB bit4->7 to control 4 digits 7 seg LED
	DDRE |= 0xC0;  	//PortE bit6,7 to control Encoder CLK Inhibit, SH/LD!
	DDRD |= 0x04;		//PortD bit2 for barGraph REG_CLK
	
}

void ADC_init(void){
	DDRF  &= ~(_BV(DDF7)); 	//make port F bit 7 the ADC input  
	PORTF &= ~(_BV(PF7));  		//port F bit 7 pullups must be off
	
	ADMUX = (1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<REFS0);    //single-ended input, PORTF bit 7, right adjusted, 
	//10 bits, reference is AVCC
	ADCSRA = (1 << ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADIE); //ADC enabled, don't start yet, enable interrupt on conversion
}

void CdSReadStart(void){
	ADCSRA |= (1 << ADSC); 			//poke the ADSC bit and start conversion
}

void setBrightness(uint8_t value){OCR2 = value;}	//Dimmest: OCR2 = 0xE1; Brightest OCR2 = 0x15;

void processLEDbrightness(uint16_t ADC_value){
	static uint8_t brightnessIndex;
	if(brightnessIndex < (ADC_value/100)) //higher index, brighter
		brightnessIndex++;
	else if(brightnessIndex > (ADC_value/100 + 1))
		brightnessIndex--;
	
	setBrightness(brightness[brightnessIndex]);
}

/* BEGIN INTERRUPT ROUTINE LIST*/
// Generating 1 second interrupt
ISR(ADC_vect){
  lastADCread = ADC; 	//Range from 50 to 1020 tested
}

ISR(TIMER0_OVF_vect){
	static uint8_t count_7ms=0;  //hold value of count between interrupts
	count_7ms++;                 //extend counter
	//toggle PB0 each time this happens
	if((count_7ms % 128) == 0) {// If one second doing something
		current_second++;
		  clear_display();
  			cursor_home();
		// blinking the mid colon everyone sec
		if(dot == 0b011)
			dot = 0b000; // blinking the mid colon
		else dot = 0b011;
	}
	if(count_7ms ==1){		//If 1792ms do something
		CdSReadStart(); 
	} 
	if((count_7ms % 32) ==0){	//if 0.25 sec doing something
		processLEDbrightness(lastADCread);
	}
	
}//TIMER0_OVF_vect


/* END INTERRUPT ROUTINE LIST*/

/***********************************************************************/
//			dirOfEncoder
//	Return: clockwise 1, counter-clockwise -1, no turn 0
/***********************************************************************/
int dirOfEncoder(void){
	//extract k bits from p position: https://www.geeksforgeeks.org/extract-k-bits-given-position-number/
	//	((1 << k) - 1) & (number >> (p - 1))
	//State machine
	
	//static variable remains in memory while the program is running
	//Almost like global except it cannot be access out of scope
	static uint8_t encoderA= 	0;
	static uint8_t encoderA_prev=	0;
	static uint8_t encoderB= 	0;
	static uint8_t encoderB_prev=	0;
	
	//save previous data
	encoderA_prev = encoderA;
	encoderB_prev = encoderB;
	
	//Update newer data
	encoderA = ((1 << 2) - 1) & (newEncodeData >> (0)); //extract bit 0 and 1 of newEncodeData
	encoderB = ((1 << 2) - 1) & (newEncodeData >> (2)); //extract bit 2 and 3 of newEncodeData
	
	//also can do
	//encoderA = (newEncodeData & 0x03);
	//encoderB = (newEncodeData & 0x0C)>>2;
	if(encoderA != encoderA_prev){
		if((encoderA_prev == 1) && (encoderA == 3))	//transistion from 3->1 is clockwise
			return 1;
		if((encoderA_prev == 2) && (encoderA == 3))
			return -1;
	}
	if(encoderB != encoderB_prev){
		if((encoderB_prev == 1) && (encoderB == 3))	//transistion from 3->1 is clockwise
			return 2;
		if((encoderB_prev == 2) && (encoderB == 3))
			return -2;
	}
	

	return 0;
}

void updateSPI(void){
	/* Encoder code */
	//Flipping bits for CLK_INH and SH/LD for encoder
	PORTE &= ~(0x40);
	PORTE |=   0x80 ;
	
	//SPDR = toBarGraph;              	//send to bar graph display 
	SPDR = 0x00;
    	while(bit_is_clear(SPSR, SPIF)){};               //wait till data sent out (a while loop)
	newEncodeData = SPDR;			//Load in new encoder data
	
	//Flipping bits for CLK_INH and SH/LD for encoder
	PORTE |=   0x40;
	PORTE &= ~(0x80);
	
	//bargraphLatch();
}


/***********************************************************************/
//update7Segment
//Convert a number to oneth, tenth ... for each digit of the 4LED - 7segment display
//No leading 0 suppression
//Parameter: Number - to print out, Dot for special character L1 L2 L3
//for Dot, pass in b001 to enable L1, b011 to enable L2+L1, b100 to enable L3
//Return: void
/***********************************************************************/
void update7Segment(uint16_t number, uint8_t dot){
				//(PORT bit = 1 in B mean 5V in electrical)
	DDRA = 0xFF;    	//Set port A to all output
	
	uint8_t oneth		= number%10;		//Extract the oneth
	uint8_t tenth		= (number%100)/10;	//Extract the tenth
	uint8_t hundredth	= (number%1000)/100;	// ...
	uint8_t thousandth	= (number%10000)/1000;
	
	//Changing output of PORTB here does not impact SPI
	//SPI need PORTB bit 1->3
	// Cycle through 4 digits of the LED display
		//PORTB = 0x00 ; //<- Issue here
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTA = indexToLed[oneth];
	_delay_ms(1);

		//PORTB = 0x10;
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTB |=  (1<<PB4);
		PORTA = indexToLed[tenth];
	_delay_ms(1);
	
	if(dot){
		//PORTB = 0x20;
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTB |=  (1<<PB5);
		// Decode for portA: 0x04 = L1+L2, 0x03 = L3||| Write a 0 to port A to turn on the dot, sinking current.
		PORTA = ~(dot);
		_delay_ms(1);
	}

		//PORTB = 0x30;
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTB |=  (1<<PB4) | (1<<PB5);
		PORTA = indexToLed[hundredth];
	_delay_ms(1);
	
		//PORTB = 0x40;
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTB |=  (1<<PB6);
		PORTA = indexToLed[thousandth];
	_delay_ms(1);
	
}


//This functional is computational intensive, major optimization can be done here
uint16_t second_to_min_hour(uint32_t second, bool mode){
	uint8_t hour_tenth;
	uint8_t hour_oneth;
	uint8_t min_tenth;
	uint8_t min_oneth;
	uint16_t result;
	
	min_oneth = ((second/60) % 60) % 10;
	min_tenth = ((second/60) % 60) / 10;
	// if mode 0 = 24 hours type, 1 = 12 hours type
	hour_oneth = (second/3600)%10;
	hour_tenth = (second/3600)/10;
	
	// if mode 0 = 24 hours type, 1 = 12 hours type
	if(hour_tenth == 2 && hour_oneth == 4) current_second = 0;	//reset second count if time reach 24 hours
	
	//Converting 24 hours to 12 hours type
	result = hour_tenth*1000 + hour_oneth*100 + min_tenth*10 + min_oneth;
	if(mode == 1){
		result  = result%1200;
	}		
	
	return result;
}

int main() {
	int flag=0;
	int state=0;
	bool isAlarmSet=0;
	
	//Setup
	spi_init();
	clock_config_init();
	IO_config_init();
	ADC_init();
	IO_config_init();
	lcd_init(); 
	clear_display();
	sei();			//Enable all interrupts
	//Loop
	
	/* Testing code */
	current_second = 0;
	dot = 0b011; // turn on the 4_LED_7_seg mid colon
	clk_mode = 1;
	/* Testing code */
	
	while(1){
		update7Segment(second_to_min_hour(current_second, clk_mode), dot); // turn on L1 and L2 also
		//update7Segment(lastADCread, dot);
		updateSPI();
		flag =  dirOfEncoder();
		
		if(flag == 2) current_second += 60;
		
		switch(state){	//State 0 Normal time with/without Alarm, State 1 setting time for alarm
			case 1:
				break;
			
			case 0:
				break;		
		}

		

  } //while
	} //main